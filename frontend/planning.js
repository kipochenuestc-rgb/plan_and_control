// Grid search demo: geometric paths, not vehicle-feasible trajectories.
function searchGrid(cols, rows, walls, start, goal, algorithm) {
 const key=(x,y)=>y*cols+x, end=key(...goal), begin=key(...start);
 const g=new Map([[begin,0]]),parent=new Map(),closed=new Set(),open=new Set([begin]);
 const heuristic=id=>algorithm==='astar'?Math.hypot(id%cols-goal[0],Math.floor(id/cols)-goal[1]):0;
 while(open.size){
  let current=-1,best=Infinity;
  for(const id of open){const cost=g.get(id)+heuristic(id);if(cost<best){best=cost;current=id}}
  open.delete(current);closed.add(current);
  if(current===end){const path=[];let id=end;while(id!==undefined){path.push([id%cols,Math.floor(id/cols)]);id=parent.get(id)}return{path:path.reverse(),visited:[...closed],length:g.get(end)}}
  const x=current%cols,y=Math.floor(current/cols);
  for(let dy=-1;dy<=1;dy++)for(let dx=-1;dx<=1;dx++){
   if(!dx&&!dy)continue;const nx=x+dx,ny=y+dy,id=key(nx,ny);
   if(nx<0||ny<0||nx>=cols||ny>=rows||walls.has(id)||closed.has(id))continue;
   if(dx&&dy&&(walls.has(key(x+dx,y))||walls.has(key(x,y+dy))))continue;
   const cost=g.get(current)+Math.hypot(dx,dy);
   if(cost<(g.get(id)??Infinity)){g.set(id,cost);parent.set(id,current);open.add(id)}
  }
 }
 return{path:[],visited:[...closed],length:0};
}
window.PlanningLab={
 template:`<main class="planning-main"><section class="planning-stage"><div class="stage-head"><b>栅格路径规划</b><span>40 × 24 · 每格 1 m</span></div><canvas ref="map" aria-label="路径规划地图，使用工具设置起终点或拖动编辑障碍" @pointerdown="begin" @pointermove="move" @pointerup="finish" @pointercancel="finish" @lostpointercapture="finish"></canvas><div class="legend planning-legend"><span class="plan-start">起点 S</span><span class="plan-goal">终点 G</span><span class="plan-wall">障碍</span><span class="plan-visited">已搜索</span><span class="actual">路径</span></div></section><aside><h2>规划控制</h2><label class="field">搜索算法<select v-model="algorithm" @change="invalidate"><option value="astar">A* · 启发式搜索</option><option value="dijkstra">Dijkstra · 最小代价搜索</option></select></label><p class="hint">{{ algorithm==='astar'?'f(n) = g(n) + h(n)，h 为到终点的直线距离。':'f(n) = g(n)，按累计代价由小到大搜索。' }}</p><label class="field">地图编辑工具</label><div class="plan-tools"><button v-for="tool in tools" :key="tool[0]" :class="{secondary:mode!==tool[0]}" :aria-pressed="mode===tool[0]" @click="mode=tool[0]">{{tool[1]}}</button></div><p class="hint">点击设置起终点；按住并拖动绘制或擦除障碍。绿色 S 为起点，橙色 G 为终点。</p><div class="buttons"><button @click="plan">搜索路径</button></div><div class="buttons plan-actions"><button class="secondary" @click="example">示例地图</button><button class="secondary" @click="clear">清空地图</button></div><p class="plan-status" role="status">{{message}}</p><div class="metrics"><div><small>路径长度</small><b>{{result.path.length?result.length.toFixed(2):'—'}} <em>m</em></b></div><div><small>已搜索格数</small><b>{{result.visited.length}}</b></div><div><small>路径点数</small><b>{{result.path.length}}</b></div><div><small>搜索耗时</small><b>{{elapsed.toFixed(2)}} <em>ms</em></b></div></div></aside><section class="notes wide"><h2>从路径搜索到自动驾驶</h2><p>地图与目标 → 路径搜索 → 车辆可行路径与速度规划 → MPC / 纵向控制 → 车辆运动</p><p>此处在二维栅格上进行八邻域搜索，禁止从两个障碍格的夹角穿过。A* 和 Dijkstra 使用相同的距离代价；比较时可观察搜索格数，耗时会受浏览器运行状态影响。</p><p>输出仅包含几何路径点 (x, y)，尚未考虑车身尺寸、最小转弯半径、航向或时间和速度约束，不能直接作为阿克曼小车的可执行轨迹。此页独立演示搜索，不改变 MPC 页的参考曲线。</p></section></main>`,
 setup(){
  const {ref,reactive,onMounted,onBeforeUnmount}=Vue,map=ref(null),algorithm=ref('astar'),mode=ref('wall'),message=ref('编辑地图后点击“搜索路径”。'),elapsed=ref(0);
  const cols=40,rows=24,walls=new Set(),start=reactive([3,12]),goal=reactive([36,12]),result=reactive({path:[],visited:[],length:0});
  let observer,drag=false,lastCell=null;
  const tools=[['start','设置起点'],['goal','设置终点'],['wall','绘制障碍'],['erase','擦除障碍']];
  function draw(){if(!map.value)return;const el=map.value,r=el.getBoundingClientRect(),d=window.devicePixelRatio||1;if(!r.width)return;el.width=r.width*d;el.height=r.height*d;const c=el.getContext('2d');c.scale(d,d);const size=Math.min(r.width/cols,r.height/rows),ox=(r.width-cols*size)/2,oy=(r.height-rows*size)/2;
   c.fillStyle='#101e30';c.fillRect(0,0,r.width,r.height);
   const cell=(id,color)=>{c.fillStyle=color;c.fillRect(ox+(id%cols)*size+1,oy+Math.floor(id/cols)*size+1,size-2,size-2)};
   result.visited.forEach(id=>cell(id,'#234c61'));walls.forEach(id=>cell(id,'#7b8a9c'));
   c.strokeStyle='#26374b';c.lineWidth=.5;for(let x=0;x<=cols;x++){c.beginPath();c.moveTo(ox+x*size,oy);c.lineTo(ox+x*size,oy+rows*size);c.stroke()}for(let y=0;y<=rows;y++){c.beginPath();c.moveTo(ox,oy+y*size);c.lineTo(ox+cols*size,oy+y*size);c.stroke()}
   if(result.path.length){c.strokeStyle='#38d3b5';c.lineWidth=3;c.beginPath();result.path.forEach(([x,y],i)=>{const px=ox+(x+.5)*size,py=oy+(y+.5)*size;i?c.lineTo(px,py):c.moveTo(px,py)});c.stroke()}
   for(const [p,label,color] of [[start,'S','#38d3b5'],[goal,'G','#ffcc72']]){cell(p[1]*cols+p[0],color);c.fillStyle='#101e30';c.font='bold '+Math.max(9,size*.65)+'px sans-serif';c.textAlign='center';c.textBaseline='middle';c.fillText(label,ox+(p[0]+.5)*size,oy+(p[1]+.5)*size)}
  }
  function invalidate(){Object.assign(result,{path:[],visited:[],length:0});elapsed.value=0;message.value='地图或算法已更新，请重新搜索。';draw()}
  function edit(e){const r=map.value.getBoundingClientRect(),size=Math.min(r.width/cols,r.height/rows),x=Math.floor((e.clientX-r.left-(r.width-cols*size)/2)/size),y=Math.floor((e.clientY-r.top-(r.height-rows*size)/2)/size);if(x<0||y<0||x>=cols||y>=rows)return;const id=y*cols+x;if(id===lastCell)return;lastCell=id;
   if(mode.value==='start'||mode.value==='goal'){const p=mode.value==='start'?start:goal,other=mode.value==='start'?goal:start;if(x===other[0]&&y===other[1])return;p[0]=x;p[1]=y;walls.delete(id)}else if(mode.value==='erase')walls.delete(id);else if(id!==start[1]*cols+start[0]&&id!==goal[1]*cols+goal[0])walls.add(id);invalidate();
  }
  function begin(e){if(e.button!==0)return;e.preventDefault();drag=true;lastCell=null;e.currentTarget.setPointerCapture(e.pointerId);edit(e)}
  function move(e){if(drag&&(mode.value==='wall'||mode.value==='erase'))edit(e)}
  function finish(){drag=false;lastCell=null}
  function plan(){const t=performance.now();Object.assign(result,searchGrid(cols,rows,walls,start,goal,algorithm.value));elapsed.value=performance.now()-t;message.value=result.path.length?'已找到最短路径。':'无可达路径，请擦除障碍或调整起终点。';draw()}
  function clear(){walls.clear();invalidate()}
  function example(){walls.clear();start.splice(0,2,3,12);goal.splice(0,2,36,12);for(let y=0;y<19;y++)walls.add(y*cols+13);for(let y=6;y<24;y++)walls.add(y*cols+26);invalidate();message.value='示例：绕过两道障碍，比较两种算法的搜索范围。'}
  onMounted(()=>{example();observer=new ResizeObserver(draw);observer.observe(map.value);draw()});onBeforeUnmount(()=>observer?.disconnect());
  return{map,algorithm,mode,tools,message,elapsed,result,begin,move,finish,plan,clear,example,invalidate};
 }
};
