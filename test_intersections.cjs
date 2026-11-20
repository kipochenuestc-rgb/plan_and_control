const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const ctx={window:{}};vm.createContext(ctx);vm.runInContext(fs.readFileSync(__dirname+'/frontend/intersections.js','utf8'),ctx);
const defaults={goal:'E',closed:false,algorithm:'dijkstra',speed:8,auto:true,J1:'NS',J2:'EW'};
assert.equal(ctx.junctionRoute('E',false).cost,200);
assert.equal(ctx.junctionRoute('E',true).cost,320);
for(const goal of ['E','N2','S2'])for(const closed of [false,true])for(const speed of [3,8,10]){
 const params={...defaults,goal,closed,speed},s=ctx.createJunctionState(params),seen=new Set();
 assert.equal(s.cost,ctx.junctionRoute(goal,closed,'astar').cost);
 const roads=vm.runInContext('junctionRoads.map(([a,b])=>[junctionNodes.find(n=>n.id===a),junctionNodes.find(n=>n.id===b)])',ctx);
 for(const p of s.points){assert(roads.some(([a,b])=>{const dx=b.x-a.x,dy=b.y-a.y,t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy)));return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy)<=5.5}), 'reference leaves the road surface');}
 for(let i=0;i<8000&&!s.done;i++){
  const passed=s.events.map(e=>e.passed);ctx.stepJunction(s,params,.05);seen.add(s.behavior);
  assert(Number.isFinite(s.pose.x)&&Number.isFinite(s.pose.y));assert(s.v>=0&&s.v<=speed+.001);
  for(let j=0;j<s.events.length;j++){const e=s.events[j];if(!passed[j]&&e.passed)assert.equal(ctx.junctionSignal(s.t,e.id,e.axis,params).color,'green');if(!e.passed)assert(s.distance<=e.stopS+.001)}
 }
 assert(s.done,JSON.stringify({goal,closed,speed,distance:s.distance,behavior:s.behavior}));assert(s.events.every(e=>e.passed));assert.equal(s.v,0);assert.equal(s.targetSpeed,0);
 if(closed){assert(!s.route.some((id,i)=>i&&s.route[i-1]==='J1'&&id==='J2'));assert(s.route.includes('N1'));}
}
const params={...defaults,auto:false,J1:'NS',J2:'NS'},waiting=ctx.createJunctionState(params);
for(let i=0;i<1000;i++)ctx.stepJunction(waiting,params,.05);
assert.equal(waiting.behavior,'WAIT');assert(waiting.distance<=waiting.events[0].stopS);assert(!waiting.events[0].passed);
params.J1='EW';for(let i=0;i<1000;i++)ctx.stepJunction(waiting,params,.05);
assert(waiting.events[0].passed);assert(!waiting.events[1].passed);assert.equal(waiting.behavior,'WAIT');
params.J2='EW';for(let i=0;i<2000&&!waiting.done;i++)ctx.stepJunction(waiting,params,.05);assert(waiting.done);
console.log('PASS: 18 route/closure/speed scenarios, A*/Dijkstra agreement, green-only stop-line crossing, manual red wait/green resume at both junctions, detour, all destinations reached');
