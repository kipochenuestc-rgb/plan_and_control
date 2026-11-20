// Exercise the real Vue setup and animation loop without a browser or API server.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const listeners = {}, docListeners = {};
let app, mount, frame;
const sandbox = {
 Vue: { ref: value=>({value}), reactive: value=>value, nextTick: fn=>Promise.resolve().then(fn),
  onMounted: fn=>mount=fn, onBeforeUnmount: ()=>{}, watch: ()=>{},
  createApp: options=>({component(){return this},mount:()=>{app=options.setup()}}) },
 window: {addEventListener:(n,f)=>listeners[n]=f},
 document: {body:{},hidden:false,addEventListener:(n,f)=>docListeners[n]=f},
 ResizeObserver: class {observe(){}}, requestAnimationFrame:fn=>(frame=fn,1),
 performance:{now:()=>0}, fetch:async()=>{throw Error('offline')},
 setTimeout,clearTimeout,console,
};
vm.runInNewContext(fs.readFileSync(__dirname+'/frontend/app.js','utf8'),sandbox);
function key(type,code,extra={}){listeners[type]({code,key:'Process',preventDefault(){},...extra})}
let clock=0;
function advance(seconds){for(let i=0;i<seconds*100;i++){clock+=10;frame(clock)}}
(async()=>{
 await mount();app.setTab(1);await Promise.resolve();
 key('keydown','KeyW');key('keydown','KeyD');advance(1);
 assert(app.kin.v>0);assert(app.kin.delta<-.1);assert(app.kin.yaw<0);
 assert(Math.abs(app.kin.inner)>Math.abs(app.kin.outer));
 key('keyup','KeyD');key('keyup','KeyW');advance(1);
 assert(Math.abs(app.kin.delta)<.01,'D key release must clear turning');
 app.resetKinematic();key('keydown','KeyA');advance(.5);
 assert(app.kin.delta>.1,'A must turn left');
 listeners.blur();advance(1);assert(Math.abs(app.kin.delta)<.01);
 app.resetKinematic();key('keydown','KeyD',{target:{matches:()=>true}});advance(.5);
 assert.equal(app.kin.delta,0,'typing in parameter fields must not steer');
 key('keydown','KeyD');advance(.5);app.setTab(0);app.setTab(1);advance(1);
 assert(Math.abs(app.kin.delta)<.01,'tab changes must clear held keys');
 app.resetKinematic();
 app.pointerDown('d',{preventDefault(){},currentTarget:{setPointerCapture(){}},pointerId:1});
 key('keydown','KeyW');advance(1);
 assert(app.kin.delta<-.1 && app.kin.yaw<0,'screen D with keyboard W must turn right');
 assert(app.pressed('d'),'screen D must highlight while held');
 app.pointerUp('d');key('keyup','KeyW');advance(1);
 assert(!app.pressed('d'));assert(Math.abs(app.kin.delta)<.01,'screen D release must center');
 console.log('PASS: W+D / IME Process / release / A / blur / input focus / tab switch / offline initialization');
 app.resetKinematic();app.kin.acceleration=1.5;
 key('keydown','KeyW');advance(1);key('keyup','KeyW');
 assert(Math.abs(app.kin.v-1.5)<.02,'W must use configured acceleration');
 app.resetKinematic();assert.equal(app.kin.acceleration,1.5,'reset must preserve acceleration');
 app.kin.acceleration=-5;key('keydown','KeyW');advance(.5);key('keyup','KeyW');
 assert.equal(app.kin.v,0,'negative acceleration setting must not reverse W');
 console.log('PASS: configurable W acceleration and bounds');
})().catch(e=>{console.error(e);process.exitCode=1});
