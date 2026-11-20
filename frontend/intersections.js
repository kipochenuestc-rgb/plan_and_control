// Small road network + route search + signal-aware behavior, without ROS or MPC.
const junctionNodes=[{id:'W',x:-60,y:0},{id:'J1',x:0,y:0},{id:'J2',x:80,y:0},{id:'E',x:140,y:0},{id:'N1',x:0,y:60},{id:'N2',x:80,y:60},{id:'S1',x:0,y:-55},{id:'S2',x:80,y:-55}];
const junctionRoads=[['W','J1'],['J1','J2'],['J2','E'],['J1','N1'],['J2','N2'],['J1','S1'],['J2','S2'],['N1','N2']];
const junctionBehaviorNames={CRUISE:'沿路线巡航',APPROACH:'转弯前减速',BRAKE:'减速至停止线',WAIT:'红灯停车等待',STRAIGHT:'通过路口',LEFT:'路口左转',RIGHT:'路口右转',ARRIVE:'终点减速',DONE:'到达终点',NO_ROUTE:'无可达路线'};
function junctionRoute(goal,closed,algorithm='dijkstra'){
 const nodes=Object.fromEntries(junctionNodes.map(n=>[n.id,n])),g={W:0},parent={},open=new Set(['W']),visited=[];
 const h=id=>algorithm==='astar'?Math.hypot(nodes[id].x-nodes[goal].x,nodes[id].y-nodes[goal].y):0;
 while(open.size){let id=[...open].sort((a,b)=>(g[a]+h(a))-(g[b]+h(b))||a.localeCompare(b))[0];open.delete(id);visited.push(id);
  if(id===goal){const route=[];let k=goal;while(k!==undefined){route.unshift(k);k=parent[k]}return{route,cost:g[goal],visited}}
  for(const[a,b]of junctionRoads){if(closed&&a==='J1'&&b==='J2')continue;const next=id===a?b:id===b?a:null;if(next===null||visited.includes(next))continue;const cost=g[id]+Math.hypot(nodes[a].x-nodes[b].x,nodes[a].y-nodes[b].y);if(cost<(g[next]??Infinity)){g[next]=cost;parent[next]=id;open.add(next)}}
 }
 return{route:[],cost:Infinity,visited};
}
function junctionGeometry(ids){
 const n=Object.fromEntries(junctionNodes.map(p=>[p.id,p])),lane=2.2,lead=10,points=[],events=[];
 const unit=(a,b)=>{const d=Math.hypot(b.x-a.x,b.y-a.y);return{x:(b.x-a.x)/d,y:(b.y-a.y)/d}},right=u=>({x:u.y*lane,y:-u.x*lane});
 const append=p=>{const last=points.at(-1);if(last&&Math.hypot(p.x-last.x,p.y-last.y)<1e-6)return;points.push({...p,s:last?last.s+Math.hypot(p.x-last.x,p.y-last.y):0})};
 function straight(p){const last=points.at(-1);if(!last){append(p);return}const steps=Math.max(1,Math.ceil(Math.hypot(p.x-last.x,p.y-last.y)/.5)),a={...last};for(let i=1;i<=steps;i++)append({x:a.x+(p.x-a.x)*i/steps,y:a.y+(p.y-a.y)*i/steps})}
 if(ids.length<2)return{points,events,length:0};
 const first=n[ids[0]],u0=unit(first,n[ids[1]]),r0=right(u0);append({x:first.x+r0.x,y:first.y+r0.y});
 for(let i=1;i<ids.length-1;i++){
  const p=n[ids[i]],u=unit(n[ids[i-1]],p),v=unit(p,n[ids[i+1]]),ru=right(u),rv=right(v),cross=u.x*v.y-u.y*v.x;
  if(Math.abs(cross)<.01)straight({x:p.x+ru.x,y:p.y+ru.y});
  else{const a={x:p.x-u.x*lead+ru.x,y:p.y-u.y*lead+ru.y},b={x:p.x+v.x*lead+rv.x,y:p.y+v.y*lead+rv.y},c={x:Math.abs(u.x)>.5?p.x+rv.x:p.x+ru.x,y:Math.abs(u.y)>.5?p.y+rv.y:p.y+ru.y};straight(a);for(let k=1;k<=32;k++){const t=k/32;append({x:(1-t)**2*a.x+2*t*(1-t)*c.x+t*t*b.x,y:(1-t)**2*a.y+2*t*(1-t)*c.y+t*t*b.y})}}
  if(ids[i]==='J1'||ids[i]==='J2')events.push({id:ids[i],axis:Math.abs(u.x)>.5?'EW':'NS',turn:cross>.1?'LEFT':cross<-.1?'RIGHT':'STRAIGHT',stopPoint:{x:p.x-u.x*14+ru.x,y:p.y-u.y*14+ru.y},exitPoint:{x:p.x+v.x*14+rv.x,y:p.y+v.y*14+rv.y},stopS:0,exitS:0,passed:false});
 }
 const end=n[ids.at(-1)],lastU=unit(n[ids.at(-2)],end),lastR=right(lastU);straight({x:end.x+lastR.x,y:end.y+lastR.y});
 const nearest=p=>points.reduce((best,q)=>Math.hypot(q.x-p.x,q.y-p.y)<Math.hypot(best.x-p.x,best.y-p.y)?q:best,points[0]).s;
 for(const e of events){e.stopS=nearest(e.stopPoint);e.exitS=nearest(e.exitPoint)}
 return{points,events,length:points.at(-1).s};
}
function junctionPose(points,distance){
 if(!points.length)return{x:0,y:0,yaw:0};let lo=0,hi=points.length-1;while(lo<hi){const mid=Math.floor((lo+hi)/2);points[mid].s<distance?lo=mid+1:hi=mid}const b=points[lo],a=points[Math.max(0,lo-1)],u=b.s>a.s?Math.max(0,Math.min(1,(distance-a.s)/(b.s-a.s))):0;
 return{x:a.x+(b.x-a.x)*u,y:a.y+(b.y-a.y)*u,yaw:Math.atan2(b.y-a.y,b.x-a.x)};
}
function junctionSignal(t,id,axis,params){
 if(!params.auto){const green=params[id]===axis;return{color:green?'green':'red',remaining:null}}
 const phase=(t+(id==='J1'?14:0))%28;
 if(axis==='EW')return phase<10?{color:'green',remaining:10-phase}:phase<12?{color:'yellow',remaining:12-phase}:{color:'red',remaining:28-phase};
 return phase>=14&&phase<24?{color:'green',remaining:24-phase}:phase>=24&&phase<26?{color:'yellow',remaining:26-phase}:{color:'red',remaining:phase<14?14-phase:42-phase};
}
function createJunctionState(params){
 const result=junctionRoute(params.goal,params.closed,params.algorithm),geometry=junctionGeometry(result.route);
 return{t:0,distance:0,v:0,a:0,targetSpeed:params.speed,behavior:result.route.length?'CRUISE':'NO_ROUTE',reason:result.route.length?'已搜索到路线，沿参考路径前往目标。':'路网不连通，请调整封路或终点。',route:result.route,cost:result.cost,visited:result.visited,points:geometry.points,events:geometry.events,length:geometry.length,pose:junctionPose(geometry.points,0),done:!result.route.length,trace:[],log:[{t:0,label:'路线规划',reason:result.route.join(' → ')+(params.closed?'；两路口间主路封闭，改走北侧连接道路。':'；按路段长度搜索最短路线。')}],nextId:'J1',stopDistance:null};
}
function stepJunction(s,params,dt){
 if(s.done)return;const clamp=(v,a,b)=>Math.max(a,Math.min(b,v)),old=s.behavior,oldV=s.v;s.t+=dt;
 const upcoming=s.events.find(e=>!e.passed),active=s.events.find(e=>e.passed&&s.distance<e.exitS);let target=params.speed,behavior='CRUISE',reason='沿已规划路线巡航，持续检查前方路口信号。',limit=null;
 s.nextId=active?.id||upcoming?.id||'终点';s.stopDistance=upcoming?Math.max(0,upcoming.stopS-s.distance):null;
 if(active){behavior=active.turn;target=Math.min(target,active.turn==='STRAIGHT'?params.speed:4);reason=`已获准通过 ${active.id}，${active.turn==='LEFT'?'完成左转':active.turn==='RIGHT'?'完成右转':'直行穿越'}并清空路口。`;}
 if(upcoming&&!active){
  const d=upcoming.stopS-s.distance,signal=junctionSignal(s.t,upcoming.id,upcoming.axis,params),green=signal.color==='green';
  // Do not rush a light that will turn yellow before the predicted stop-line arrival.
  const expiring=green&&signal.remaining!==null&&signal.remaining<4&&d/Math.max(s.v,1)+.8>signal.remaining;
  if((!green||expiring)&&d<Math.max(38,s.v*s.v/4+12)){
   limit=Math.max(0,d);target=0;behavior=s.v<.06?'WAIT':'BRAKE';reason=`${upcoming.id} ${upcoming.axis==='EW'?'东西':'南北'}向${expiring?'绿灯即将结束':signal.color==='yellow'?'黄灯':'红灯'}，${behavior==='WAIT'?'在停止线前等待放行':'减速至停止线前'}。`;
  }else if(upcoming.turn!=='STRAIGHT'&&d<30){target=Math.min(target,4);behavior='APPROACH';reason=`路线要求在 ${upcoming.id} ${upcoming.turn==='LEFT'?'左转':'右转'}，提前将目标速度降至 4 m/s。`;}
 }
 const end=Math.max(0,s.length-s.distance);
 if(end<Math.max(10,s.v*s.v/4+8)&&(limit===null||end<limit)){limit=end;target=0;behavior='ARRIVE';reason='接近目标位置，减速停车。'}
 s.targetSpeed=target;s.behavior=behavior;s.reason=reason;
 if(limit!==null){s.a=limit>.025?clamp(-s.v*s.v/(2*limit),-3.5,0):0;s.v=s.v<.025||limit<.025?0:Math.max(0,s.v+s.a*dt)}else{s.a=clamp(1.3*(target-s.v),-3.5,2);s.v=Math.max(0,s.v+s.a*dt)}
 let dx=(oldV+s.v)*dt*.5;if(limit!==null)dx=Math.min(dx,limit);
 // Ideal execution guard: never cross an unpermitted stop line, even after a manual signal change.
 if(upcoming&&!upcoming.passed&&s.distance+dx>=upcoming.stopS-1e-6){const light=junctionSignal(s.t,upcoming.id,upcoming.axis,params);if(light.color==='green'&&limit===null){upcoming.passed=true;s.log.unshift({t:s.t,label:'路口放行',reason:`${upcoming.id} ${upcoming.axis==='EW'?'东西':'南北'}向绿灯，获得通行许可。`})}else{dx=Math.max(0,upcoming.stopS-s.distance);s.v=0}}
 s.distance=Math.min(s.length,s.distance+dx);s.pose=junctionPose(s.points,s.distance);
 if(end<.06&&s.v<.06){s.done=true;s.v=0;s.a=0;s.targetSpeed=0;s.behavior='DONE';s.reason='已沿规划路线通过路口并到达目标。'}
 if(s.behavior!==old)s.log.unshift({t:s.t,label:junctionBehaviorNames[s.behavior],reason:s.reason});s.log=s.log.slice(0,18);s.trace.push([s.pose.x,s.pose.y]);if(s.trace.length>2500)s.trace.shift();
}
window.IntersectionsLab={
 template:`<main class="junction-main"><section class="junction-stage"><div class="stage-head"><b>双十字路口 · 路线与行为联合仿真</b><span>{{running?'运行中':'已暂停'}} · {{s.t.toFixed(1)}} s</span></div><canvas ref="map" aria-label="两个十字路口的路线搜索、信号灯与车辆行为综合仿真"></canvas><div class="graph-legend"><span><i class="g-path"></i>自车 / 已行驶</span><span><i class="g-open"></i>规划路线</span><span><i class="g-current"></i>图节点 / 道路</span><label><input type="checkbox" v-model="showGraph"> 显示路网图</label></div><div class="graph-message" role="status"><b>{{names[s.behavior]}} · 下一目标 {{s.nextId}}</b><p>{{s.reason}}</p><p class="behavior-output">路线：{{s.route.join(' → ')}}</p></div></section><aside><h2>综合仿真控制</h2><div class="buttons"><button @click="toggle" :disabled="s.done">{{running?'暂停':'开始'}}</button><button class="secondary" @click="single" :disabled="running||s.done">单步 0.2 s</button></div><div class="buttons plan-actions"><button class="secondary" @click="reset">重置并重新规划</button></div><label class="field">目的地<select v-model="params.goal" @change="reset"><option value="E">E · 东侧出口（直行）</option><option value="N2">N2 · 北侧出口（左转）</option><option value="S2">S2 · 南侧出口（右转）</option></select></label><label class="field">路线搜索<select v-model="params.algorithm" @change="reset"><option value="dijkstra">Dijkstra</option><option value="astar">A*</option></select></label><label class="behavior-check"><input type="checkbox" v-model="params.closed" @change="reset"> 封闭 J1 ↔ J2，搜索绕行路线</label><p class="hint">更改终点、搜索算法或封路状态会重置仿真。北侧连接道路提供绕行通道。</p><label class="slider">巡航目标速度<strong>{{params.speed}} m/s</strong><input type="range" min="3" max="10" step="0.5" v-model.number="params.speed"></label><div class="metrics"><div><small>实际速度</small><b>{{s.v.toFixed(2)}} <em>m/s</em></b></div><div><small>行为目标速度</small><b>{{s.targetSpeed.toFixed(1)}} <em>m/s</em></b></div><div><small>距下一停止点</small><b>{{s.stopDistance===null?'—':s.stopDistance.toFixed(1)}} <em>m</em></b></div><div><small>路线进度</small><b>{{s.length?(100*s.distance/s.length).toFixed(0):0}} <em>%</em></b></div></div><p class="hint">搜索代价（路段中心线长度）：{{s.cost.toFixed(0)}} m<br>平滑参考路径长度：{{s.length.toFixed(1)}} m</p></aside><section class="notes junction-signals"><div class="stage-head"><h2>路口信号与通行许可</h2><label><input type="checkbox" v-model="params.auto"> 自动切换信号</label></div><p class="hint">自动模式含绿灯、黄灯与全红间隔；每周期 28 s，两个路口错开 14 s。已获准通行的车辆继续清空路口。</p><div class="junction-light-grid"><div v-for="id in ['J1','J2']" :key="id" class="junction-light-card"><h2>{{id}} · {{id==='J1'?'左侧十字路口':'右侧十字路口'}}</h2><div v-for="axis in ['EW','NS']" :key="axis" class="junction-light-row"><span>{{axis==='EW'?'东西方向':'南北方向'}}</span><b :class="'light-'+signal(id,axis).color">{{colorName(signal(id,axis).color)}}{{signal(id,axis).remaining===null?'':' · '+signal(id,axis).remaining.toFixed(1)+' s'}}</b></div><div v-if="!params.auto" class="buttons plan-actions"><button class="secondary" @click="params[id]='EW'">东西放行</button><button class="secondary" @click="params[id]='NS'">南北放行</button></div><p class="hint">{{eventText(id)}}</p></div></div></section><section class="notes wide"><h2>路径规划与行为规划如何配合</h2><div class="behavior-pipeline"><span>路网与目的地<br><small>节点、路段与封路约束</small></span><b>→</b><span>{{params.algorithm==='astar'?'A*':'Dijkstra'}} 路线搜索<br><small>选择经过哪些路段</small></span><b>→</b><span>局部几何路径<br><small>右侧车道偏移 + 平滑转弯</small></span><b>→</b><span>行为状态机<br><small>巡航 / 等灯 / 通过 / 转弯</small></span><b>→</b><span>示意车辆执行<br><small>沿参考路径前进</small></span></div><p>两个十字路口由东西主路连接，北侧另有绕行道路。封闭主路后重新搜索路线，车辆会改走北侧道路，并在路口按所选路线决定直行、左转或右转。</p><p>行为规划每 0.05 s 检查路线上的下一路口与来车方向信号：红灯或黄灯减速停车，绿灯允许通行；转弯目标速度不超过 4 m/s。车辆中心在路口前约 14 m 停止，车头位于白色停止线之前。</p><small>本页将路径搜索与行为决策联合演示。车辆沿平滑路径弧长作理想执行，未接入现有 MPC，也未模拟横向动力学、对向让行或其他交通参与者。停线约束会直接限制运动，因此手动突变信号不代表真实车辆的紧急制动过程。信号时间均为仿真时间。</small><h2>路线与行为事件</h2><div class="behavior-log"><div v-for="(item,i) in s.log" :key="i"><time>{{item.t.toFixed(1)}} s</time><b>{{item.label}}</b><span>{{item.reason}}</span></div></div></section></main>`,
 setup(){
  const{ref,reactive,computed,watch,onMounted,onBeforeUnmount}=Vue,params=reactive({goal:'E',closed:false,algorithm:'dijkstra',speed:8,auto:true,J1:'NS',J2:'EW'}),s=reactive(createJunctionState(params)),map=ref(null),running=ref(false),showGraph=ref(true);
  let frame=null,observer=null,last=null,accumulator=0;
  const signal=(id,axis)=>junctionSignal(s.t,id,axis,params),colorName=color=>({green:'绿灯',yellow:'黄灯',red:'红灯'}[color]);
  function eventText(id){const e=s.events.find(e=>e.id===id);return !e?'本次路线不经过此路口':e.passed?'已获通行许可 / 已通过':`将从${e.axis==='EW'?'东西':'南北'}方向进入，${e.turn==='LEFT'?'左转':e.turn==='RIGHT'?'右转':'直行'}`}
  function draw(){
   if(!map.value)return;const r=map.value.getBoundingClientRect();if(!r.width)return;const el=map.value,d=window.devicePixelRatio||1;el.width=r.width*d;el.height=r.height*d;const c=el.getContext('2d');c.scale(d,d);const scale=Math.min((r.width-36)/235,(r.height-46)/150),pt=(x,y)=>[r.width/2+(x-40)*scale,r.height/2-(y-3)*scale],byId=Object.fromEntries(junctionNodes.map(n=>[n.id,n]));
   c.fillStyle='#101e30';c.fillRect(0,0,r.width,r.height);
   function segment(a,b,color,width,dash=[]){c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.beginPath();c.moveTo(...pt(a.x,a.y));c.lineTo(...pt(b.x,b.y));c.stroke();c.setLineDash([])}
   c.lineCap='butt';for(const[a,b]of junctionRoads)segment(byId[a],byId[b],'#8495a7',11.8*scale);for(const[a,b]of junctionRoads)segment(byId[a],byId[b],'#293a50',11*scale);for(const[a,b]of junctionRoads)segment(byId[a],byId[b],'#b7c4cf',1,[7,7]);
   // Repaint junction squares so road markings do not imply lane priority.
   for(const id of ['J1','J2','N1','N2']){const n=byId[id],p=pt(n.x-5.5,n.y+5.5);c.fillStyle='#293a50';c.fillRect(p[0],p[1],11*scale,11*scale)}
   for(const id of ['J1','J2']){const n=byId[id];for(const[u,v]of [[1,0],[-1,0],[0,1],[0,-1]]){const a={x:n.x-u*11+v*.4,y:n.y-v*11-u*.4},b={x:n.x-u*11+v*5,y:n.y-v*11-u*5};segment(a,b,'#f0f4f8',2);for(let k=-4.4;k<5;k+=1.65)segment({x:n.x-u*7.3+v*k,y:n.y-v*7.3-u*k},{x:n.x-u*9.3+v*k,y:n.y-v*9.3-u*k},'#aebcc9',.65*scale)}}
   if(params.closed){const a=pt(34,5.4),b=pt(46,-5.4);c.fillStyle='#af5d52';c.fillRect(a[0],a[1],b[0]-a[0],b[1]-a[1]);c.strokeStyle='#ffd2a5';c.lineWidth=2;for(let x=34;x<46;x+=3)segment({x,y:-5},{x:x+2,y:5},'#ffd2a5',2);c.fillStyle='#ffccc3';c.font='11px sans-serif';c.textAlign='center';c.fillText('主路封闭',pt(40,-12)[0],pt(40,-12)[1])}
   if(showGraph.value){for(const[a,b]of junctionRoads){if(params.closed&&a==='J1'&&b==='J2')continue;segment(byId[a],byId[b],'rgba(255,204,114,.4)',1,[3,5])}for(const n of junctionNodes){const p=pt(n.x,n.y);c.fillStyle='#ffcc72';c.beginPath();c.arc(...p,3,0,2*Math.PI);c.fill();c.font='bold 11px sans-serif';c.textAlign='left';c.fillText(n.id,p[0]+7,p[1]-8)}}
   function poly(points,color,width,dash=[]){if(!points.length)return;c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.beginPath();points.forEach((p,i)=>{const q=pt(p.x??p[0],p.y??p[1]);i?c.lineTo(...q):c.moveTo(...q)});c.stroke();c.setLineDash([])}
   poly(s.points,'#86b7ff',2.5,[6,4]);poly(s.trace,'#38d3b5',3);
   for(const id of ['J1','J2']){const n=byId[id],p=pt(n.x+10,n.y+15);c.fillStyle='#0b1525';c.fillRect(p[0]-6,p[1]-12,67,29);c.font='10px sans-serif';c.textAlign='left';c.fillStyle={red:'#f58c8c',yellow:'#ffcc72',green:'#38d3b5'}[signal(id,'EW').color];c.fillText('东西 ●',p[0],p[1]);c.fillStyle={red:'#f58c8c',yellow:'#ffcc72',green:'#38d3b5'}[signal(id,'NS').color];c.fillText('南北 ●',p[0],p[1]+12)}
   const end=s.points.at(-1);if(end){const p=pt(end.x,end.y);c.strokeStyle='#38d3b5';c.lineWidth=2;c.beginPath();c.arc(...p,7,0,Math.PI*2);c.stroke();c.fillStyle='#c5f5e8';c.font='11px sans-serif';c.textAlign='center';c.fillText('终点',p[0],p[1]+24)}
   const p=pt(s.pose.x,s.pose.y),carScale=Math.max(scale,4.5);c.save();c.translate(...p);c.rotate(-s.pose.yaw);c.fillStyle='#38d3b5';c.strokeStyle='#b8ffec';c.lineWidth=1;c.fillRect(-2.25*carScale,-.9*carScale,4.5*carScale,1.8*carScale);c.strokeRect(-2.25*carScale,-.9*carScale,4.5*carScale,1.8*carScale);c.fillStyle='#173950';c.fillRect(.5*carScale,-.7*carScale,.7*carScale,1.4*carScale);c.restore();c.fillStyle='#b8ffec';c.textAlign='center';c.font='bold 11px sans-serif';c.fillText('自车',p[0],p[1]-13);
   c.fillStyle='#a7bdcf';c.textAlign='left';c.font='12px sans-serif';c.fillText('北 ↑ · 右侧通行 · 两路口相距 80 m · 车身放大显示',16,r.height-16);c.fillStyle='#e5f0f7';c.font='bold 13px sans-serif';c.fillText(junctionBehaviorNames[s.behavior],16,26);
  }
  function reset(){running.value=false;accumulator=0;Object.assign(s,createJunctionState(params));draw()}
  function advance(){stepJunction(s,params,.05);if(s.done)running.value=false}
  function single(){for(let i=0;i<4;i++)advance();draw()}
  function toggle(){running.value=!running.value;last=null;accumulator=0}
  function animate(now){const dt=last===null?0:Math.min(.1,(now-last)/1000);last=now;if(running.value){accumulator+=dt;while(accumulator>=.05&&running.value){advance();accumulator-=.05}draw()}frame=requestAnimationFrame(animate)}
  watch(()=>[params.auto,params.J1,params.J2,showGraph.value],draw);
  onMounted(()=>{observer=new ResizeObserver(draw);observer.observe(map.value);draw();frame=requestAnimationFrame(animate)});onBeforeUnmount(()=>{cancelAnimationFrame(frame);observer?.disconnect()});
  return{params,s,map,running,showGraph,signal,colorName,eventText,names:junctionBehaviorNames,toggle,single,reset};
 }
};
