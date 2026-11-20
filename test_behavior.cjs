const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const ctx={window:{}};vm.createContext(ctx);
vm.runInContext(fs.readFileSync(__dirname+'/frontend/behavior.js','utf8'),ctx);
for(const speed of [3,8,12])for(const scenario of ['cruise','follow','obstacle','blocked','red']){
 const s=ctx.createBehaviorState(scenario),params={speed,allowChange:true},seen=new Set();let minGap=Infinity;
 for(let i=0;i<2000&&!s.done;i++){
  ctx.stepBehavior(s,params,.05);seen.add(s.behavior);
  assert(s.a>=-3.50001&&s.a<=2.00001);assert(s.v>=0&&s.v<=12);
  assert(Number.isFinite(s.x)&&Number.isFinite(s.y));
  if(scenario==='follow')minGap=Math.min(minGap,s.leadX-s.x-4.5);
  if(scenario==='obstacle'&&Math.abs(s.y)<1.8)assert(Math.abs(s.x-s.obstacleX)>4.5,'vehicle overlaps obstacle');
 }
 if(scenario==='blocked'){assert(s.x<=70.6);assert(s.v<.08);assert(seen.has('WAIT'));}
 if(scenario==='red'){
  assert(s.x<=91.01);assert(s.v<.08);assert(seen.has('WAIT'));
  s.red=false;for(let i=0;i<100;i++)ctx.stepBehavior(s,params,.05);assert(s.x>95,'green light must resume motion');
 }
 if(scenario==='follow')assert(minGap>4,'following clearance');
 if(scenario==='obstacle'){assert(seen.has('CHANGE_LEFT'));assert(seen.has('CHANGE_RIGHT'));assert.equal(s.lane,0);}
 if(['cruise','follow','obstacle'].includes(scenario)){assert(s.done);assert.equal(s.targetSpeed,0);assert(s.x<=160.001);}
}
const disabled=ctx.createBehaviorState('obstacle');
for(let i=0;i<1200;i++)ctx.stepBehavior(disabled,{speed:8,allowChange:false},.05);
assert.equal(disabled.lane,0);assert.equal(disabled.behavior,'WAIT');assert(disabled.x<71);
console.log('PASS: five scenarios at 3/8/12 m/s, acceleration bounds, obstacle clearance, return lane, red stop/green resume, goal stop, disabled lane changes');
