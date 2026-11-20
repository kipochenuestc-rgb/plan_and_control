// Educational rule-based behavior planner. Motion is deliberately independent of MPC.
const behaviorLabels={CRUISE:'巡航',FOLLOW:'跟车',DECEL:'减速停车',WAIT:'停车等待',CHANGE_LEFT:'向左变道',CHANGE_RIGHT:'返回原车道',DONE:'到达终点'};
function createBehaviorState(scenario='obstacle'){
 return{scenario,t:0,x:0,y:0,v:0,a:0,lane:0,targetLane:0,targetSpeed:8,behavior:'CRUISE',reason:'道路前方畅通，沿当前车道巡航。',change:null,leadX:40,leadV:3,obstacleX:80,stopX:95,red:true,done:false,log:[],trace:[],gap:Infinity};
}
function stepBehavior(s,params,dt){
 if(s.done)return;
 const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),cruise=clamp(Number(params.speed)||0,0,12),old=s.behavior,oldV=s.v;
 s.t+=dt;if(s.scenario==='follow')s.leadX+=s.leadV*dt;
 const obstacle=s.scenario==='obstacle'||s.scenario==='blocked',adjacent=s.scenario!=='blocked'&&params.allowChange,obstacleGap=s.obstacleX-s.x-4.5;
 let desired=cruise,behavior='CRUISE',reason='道路前方畅通，沿当前车道巡航。',stopDistance=null;
 s.gap=Infinity;
 function change(to){s.change={from:s.lane,to,elapsed:0,duration:3.2};s.targetLane=to}
 if(!s.change&&obstacle&&s.lane===0&&obstacleGap>0&&obstacleGap<Math.max(42,s.v*4.5)&&adjacent)change(1);
 if(!s.change&&obstacle&&s.lane===1&&s.x>s.obstacleX+19)change(0);
 if(s.change){behavior=s.change.to===1?'CHANGE_LEFT':'CHANGE_RIGHT';desired=Math.min(cruise,6);reason=s.change.to===1?'当前车道有静态障碍，邻道可用；执行已确认的向左变道。':'已越过障碍并留出车身余量，返回原车道。';}
 else if(s.scenario==='red'&&s.red&&s.x<s.stopX&&s.stopX-s.x-4<Math.max(50,s.v*s.v/4+12)){
  stopDistance=Math.max(0,s.stopX-s.x-4);desired=0;behavior=s.v<.08?'WAIT':'DECEL';reason=s.v<.08?'红灯仍亮，保持停车；切换绿灯后恢复巡航。':'红灯停车优先，规划在停止线前 4 m 停下（车辆中心）。';s.gap=s.stopX-s.x-2.25;
 }else if(obstacle&&s.lane===0&&obstacleGap>=0&&obstacleGap<Math.max(50,s.v*s.v/4+12)){
  stopDistance=Math.max(0,obstacleGap-5);desired=0;behavior=s.v<.08?'WAIT':'DECEL';reason=s.v<.08?'前方障碍未清除，保持 5 m 净距并等待。':(adjacent?'接近障碍，等待满足变道触发条件。':'前方障碍且邻道不可用或禁止变道，减速并保留 5 m 净距。');s.gap=obstacleGap;
 }else if(s.scenario==='follow'){
  const gap=s.leadX-s.x-4.5,desiredGap=6+1.4*s.v;s.gap=gap;
  if(gap<Math.max(35,desiredGap+15)){desired=clamp(s.leadV+.4*(gap-desiredGap),0,cruise);behavior='FOLLOW';reason='前车较慢，按“6 m + 1.4 s × 自车速度”的期望净距跟车。';}
 }
 const goalDistance=Math.max(0,160-s.x);
 if(goalDistance<Math.max(8,s.v*s.v/4+8)&&(stopDistance===null||goalDistance<stopDistance)){
  stopDistance=goalDistance;desired=0;behavior='DECEL';reason='接近演示终点，减速停车。';
 }
 s.targetSpeed=desired;s.behavior=behavior;s.reason=reason;
 if(stopDistance!==null){s.a=stopDistance>.03?clamp(-s.v*s.v/(2*stopDistance),-3.5,0):0;if(s.v<.03||stopDistance<.03){s.v=0;s.a=0}else s.v=Math.max(0,s.v+s.a*dt)}
 else{s.a=clamp(1.3*(desired-s.v),-3.5,2);s.v=clamp(s.v+s.a*dt,0,12)}
 const dx=(oldV+s.v)*.5*dt;s.x+=stopDistance===null?dx:Math.min(dx,stopDistance);
 if(s.change){s.change.elapsed+=dt;const p=clamp(s.change.elapsed/s.change.duration,0,1),blend=10*p**3-15*p**4+6*p**5;s.y=3.6*(s.change.from+(s.change.to-s.change.from)*blend);if(p===1){s.lane=s.change.to;s.targetLane=s.lane;s.change=null}}
 else{s.targetLane=s.lane;s.y=s.lane*3.6}
 if(s.x>=159.95&&s.v<.08){s.done=true;s.v=0;s.a=0;s.targetSpeed=0;s.behavior='DONE';s.reason='本场景演示结束。可重置或选择其他场景。'}
 if(s.behavior!==old){s.log.unshift({t:s.t,label:behaviorLabels[s.behavior],reason:s.reason});if(s.log.length>12)s.log.pop()}
 s.trace.push([s.x,s.y]);if(s.trace.length>1600)s.trace.shift();
}
window.BehaviorLab={
 template:`<main class="behavior-main"><section class="behavior-stage"><div class="stage-head"><b>行为规划 · 双车道场景</b><span>{{running?'运行中':'已暂停'}} · {{s.t.toFixed(1)}} s</span></div><canvas ref="map" aria-label="行为规划仿真：双车道上的自车、前车、障碍和停止线"></canvas><div class="graph-legend"><span><i class="g-path"></i>自车 / 实际路线</span><span><i class="g-current"></i>前车 / 障碍</span><span><i class="g-open"></i>局部参考路径</span><span>两车道同向 →</span></div><div class="behavior-flow"><div v-for="id in states" :key="id" :class="{selected:s.behavior===id}">{{labels[id]}}</div></div><div class="graph-message" role="status"><b>当前决策：{{labels[s.behavior]}}</b><p>{{s.reason}}</p><p class="behavior-output">输出：目标车道 {{s.targetLane===0?'右车道':'左车道'}} · 目标速度 {{s.targetSpeed.toFixed(1)}} m/s</p></div></section><aside><h2>行为场景</h2><label class="field">场景选择<select v-model="scenario" @change="reset"><option value="cruise">1 · 畅通道路：巡航</option><option value="follow">2 · 慢速前车：跟车</option><option value="obstacle">3 · 前方障碍：变道绕行</option><option value="blocked">4 · 邻道封闭：停车等待</option><option value="red">5 · 红绿灯：停车 / 起步</option></select></label><p class="hint">切换场景会重置车辆。点击“开始”观察行为选择与切换原因。</p><div class="buttons"><button @click="toggle" :disabled="s.done">{{running?'暂停':'开始'}}</button><button class="secondary" @click="single" :disabled="running||s.done">单步 0.2 s</button></div><div class="buttons plan-actions"><button class="secondary" @click="reset">重置场景</button></div><label class="slider">巡航目标速度<strong>{{params.speed}} m/s</strong><input type="range" min="3" max="12" step="0.5" v-model.number="params.speed"></label><label class="behavior-check"><input type="checkbox" v-model="params.allowChange"> 允许变道（执行中的变道会完成）</label><p v-if="scenario==='blocked'" class="hint">左侧车道封闭，即使允许变道也无法绕行。</p><button v-if="scenario==='red'" class="behavior-signal" :class="{red:s.red}" @click="s.red=!s.red">当前{{s.red?'红灯':'绿灯'}} · 点击切换</button><div class="metrics"><div><small>实际速度</small><b>{{s.v.toFixed(2)}} <em>m/s</em></b></div><div><small>实际加速度</small><b>{{s.a.toFixed(2)}} <em>m/s²</em></b></div><div><small>前方净距</small><b>{{Number.isFinite(s.gap)?s.gap.toFixed(1):'—'}} <em>m</em></b></div><div><small>自车位置</small><b>{{s.x.toFixed(1)}} <em>m</em></b></div></div><div class="solver">邻道状态 <b>{{scenario==='blocked'?'封闭':'可用'}}</b><br>纵向加速度限制 <b>−3.5 ～ 2 m/s²</b></div></aside><section class="notes wide"><h2>这个 Tab 怎样仿真行为规划</h2><div class="behavior-pipeline"><span>场景输入<br><small>前车 / 障碍 / 信号灯 / 邻道</small></span><b>→</b><span>规则与状态机<br><small>巡航 / 跟车 / 停车 / 变道</small></span><b>→</b><span>决策输出<br><small>目标车道 + 目标速度</small></span><b>→</b><span>简化运动执行<br><small>速度响应 + 平滑换道</small></span></div><p>行为规划决定“做什么”；局部路径与速度规划决定“具体怎样走”；控制器负责跟踪。这里用规则状态机展示决策，以平滑横向插值和受加速度限制的纵向运动显示结果，尚未连接现有 MPC。</p><p>跟车期望净距：d = 6 + 1.4v（m）。静态障碍停车保留 5 m 净距。变道用时 3.2 s，变道目标速度不高于 6 m/s。左侧车道封闭时禁止绕行。</p><small>这是教学演示，并非 Autoware 源码移植。未模拟感知误差、动态邻车间隙评估、紧急障碍或完整车辆动力学；图上的蓝色曲线为简化局部参考路径。</small><h2>行为切换记录</h2><div class="behavior-log"><div v-for="(item,i) in s.log" :key="i"><time>{{item.t.toFixed(1)}} s</time><b>{{item.label}}</b><span>{{item.reason}}</span></div><p v-if="!s.log.length">尚未切换行为，开始后会记录每次决策变化。</p></div></section></main>`,
 setup(){
  const{ref,reactive,onMounted,onBeforeUnmount,watch}=Vue,scenario=ref('obstacle'),s=reactive(createBehaviorState()),params=reactive({speed:8,allowChange:true}),map=ref(null),running=ref(false),states=['CRUISE','FOLLOW','DECEL','WAIT','CHANGE_LEFT','CHANGE_RIGHT'];
  let frame=null,observer=null,last=null,accumulator=0;
  function draw(){if(!map.value)return;const r=map.value.getBoundingClientRect(),d=window.devicePixelRatio||1;if(!r.width)return;const el=map.value;el.width=r.width*d;el.height=r.height*d;const c=el.getContext('2d');c.scale(d,d);const scale=r.width/100,cx=s.x+34,cy=r.height*.53,pt=(x,y)=>[(x-cx)*scale+r.width/2,cy-y*scale];
   c.fillStyle='#101e30';c.fillRect(0,0,r.width,r.height);const roadTop=pt(0,5.4)[1],roadBottom=pt(0,-1.8)[1];c.fillStyle='#233247';c.fillRect(0,roadTop,r.width,roadBottom-roadTop);
   for(const y of [-1.8,1.8,5.4]){c.strokeStyle='#9eacbc';c.lineWidth=y===1.8?1.5:2;c.setLineDash(y===1.8?[13,12]:[]);c.beginPath();c.moveTo(0,pt(0,y)[1]);c.lineTo(r.width,pt(0,y)[1]);c.stroke()}c.setLineDash([]);
   c.font='12px sans-serif';c.fillStyle='#9bb0c5';for(let x=Math.floor((s.x-20)/20)*20;x<s.x+90;x+=20)c.fillText(x+' m',pt(x,0)[0],roadBottom+28);
   c.fillStyle='#a3b5c8';c.fillText('左车道 →',14,roadTop-18);c.fillText('右车道 →',14,roadBottom+53);
   if(s.scenario==='blocked'){const a=pt(35,5.3),b=pt(120,1.9);c.fillStyle='rgba(235,112,99,.22)';c.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);c.strokeStyle='#e48777';for(let x=35;x<120;x+=5){const p=pt(x,5.2),q=pt(x+2,2);c.beginPath();c.moveTo(...p);c.lineTo(...q);c.stroke()}c.fillStyle='#ffd0c8';c.fillText('邻道封闭',pt(48,3.6)[0],pt(48,3.6)[1])}
   function vehicle(x,y,color,label){const p=pt(x,y);c.fillStyle=color;c.fillRect(p[0]-2.25*scale,p[1]-.9*scale,4.5*scale,1.8*scale);c.fillStyle='#123044';c.fillRect(p[0]+.6*scale,p[1]-.65*scale,.7*scale,1.3*scale);c.fillStyle='#e3eef9';c.font='12px sans-serif';c.fillText(label,p[0]-10,p[1]-1.5*scale)}
   if(s.scenario==='follow')vehicle(s.leadX,0,'#ffcc72','前车 3 m/s');
   if(s.scenario==='obstacle'||s.scenario==='blocked'){vehicle(s.obstacleX,0,'#e9a376','障碍');const p=pt(s.obstacleX,0);c.strokeStyle='#6f341c';c.beginPath();c.moveTo(p[0]-scale,p[1]-scale*.6);c.lineTo(p[0]+scale,p[1]+scale*.6);c.moveTo(p[0]-scale,p[1]+scale*.6);c.lineTo(p[0]+scale,p[1]-scale*.6);c.stroke()}
   if(s.scenario==='red'){const p=pt(s.stopX,5.4),q=pt(s.stopX,-1.8);c.strokeStyle='#eef4fa';c.lineWidth=3;c.beginPath();c.moveTo(...p);c.lineTo(...q);c.stroke();c.fillStyle=s.red?'#f17d7d':'#38d3b5';c.beginPath();c.arc(p[0],p[1]-22,9,0,Math.PI*2);c.fill();c.fillText(s.red?'红灯':'绿灯',p[0]+14,p[1]-18)}
   function poly(points,color,dash=[]){if(!points.length)return;c.strokeStyle=color;c.lineWidth=2;c.setLineDash(dash);c.beginPath();points.forEach((p,i)=>{const q=pt(...p);i?c.lineTo(...q):c.moveTo(...q)});c.stroke();c.setLineDash([])}
   poly(s.trace,'#38d3b5');const preview=[];for(let i=0;i<=30;i++){const future=i*.1;let y=s.y;if(s.change){const p=Math.min(1,(s.change.elapsed+future)/s.change.duration),b=10*p**3-15*p**4+6*p**5;y=3.6*(s.change.from+(s.change.to-s.change.from)*b)}preview.push([s.x+Math.max(.1,s.v)*future,y])}poly(preview,'#86b7ff',[5,5]);vehicle(s.x,s.y,'#38d3b5','自车');
   const end=pt(160,-1.8);c.fillStyle='#e8f1f7';if(end[0]>0&&end[0]<r.width)c.fillText('终点 160 m',end[0]-20,roadBottom+72);
   c.fillStyle='#d4e2f0';c.font='bold 14px sans-serif';c.fillText(behaviorLabels[s.behavior],18,30);c.fillStyle='#8fa6bd';c.font='12px sans-serif';c.fillText('示意运动 · 车道宽 3.6 m · 镜头跟随自车',18,r.height-18);
  }
  function reset(){running.value=false;accumulator=0;Object.assign(s,createBehaviorState(scenario.value));s.targetSpeed=params.speed;draw()}
  function advance(dt){stepBehavior(s,params,dt);if(s.done)running.value=false}
  function single(){for(let i=0;i<4;i++)advance(.05);draw()}
  function toggle(){running.value=!running.value;last=null;accumulator=0}
  function animate(now){const dt=last===null?0:Math.min(.1,(now-last)/1000);last=now;if(running.value){accumulator+=dt;while(accumulator>=.05&&running.value){advance(.05);accumulator-=.05}draw()}frame=requestAnimationFrame(animate)}
  watch(()=>s.red,draw);
  onMounted(()=>{observer=new ResizeObserver(draw);observer.observe(map.value);draw();frame=requestAnimationFrame(animate)});onBeforeUnmount(()=>{cancelAnimationFrame(frame);observer?.disconnect()});
  return{scenario,s,params,map,running,states,labels:behaviorLabels,toggle,reset,single,Number};
 }
};
