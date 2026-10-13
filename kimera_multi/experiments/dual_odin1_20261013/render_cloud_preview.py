"""Render and export an interactive, self-contained point cloud preview."""
import base64
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'artifacts/dual_odin1_20261013/cloud_fusion_preview'
report = json.loads((OUT / 'registration.json').read_text())
font_path = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font_path)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font_path).get_name(),
                     'axes.unicode_minus':False,'font.size':11,'figure.facecolor':'#f4f7fb'})
rng = np.random.default_rng(42)
maps = {}
for mode in ['sdk','vio']:
    data = np.load(OUT / f'fused_{mode}_candidate.npz')
    maps[mode] = {name:data[name] for name in ['a','b']}
all_points = np.concatenate([points for m in maps.values() for points in m.values()])
low, high = all_points.min(axis=0), all_points.max(axis=0)
low -= .2; high += .2
fig = plt.figure(figsize=(13,10.6))
fig.suptitle('双 Odin1 点云｜离线几何配准预览',x=.06,y=.98,ha='left',fontsize=22,fontweight='bold')
fig.text(.06,.94,'蓝色 = Odin A    粉色 = Odin B    ·    5 cm 体素    ·    尚非 Kimera-Multi 跨机融合',color='#435568',fontsize=12)
stats = []
for col, mode in enumerate(['sdk','vio']):
    points = maps[mode]
    title = 'SDK 参考轨迹累积' if mode=='sdk' else 'Kimera VIO 轨迹累积'
    ax = fig.add_subplot(2,2,col+1,projection='3d')
    for name,color in [('a','#1687cd'),('b','#cd5ab4')]:
        cloud=points[name]
        sample=cloud[rng.choice(len(cloud),min(22000,len(cloud)),replace=False)]
        ax.scatter(*sample.T,c=color,s=.55,alpha=.55,rasterized=True,depthshade=False)
    ax.set(title=title,xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)')
    ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_zlim(low[2],high[2])
    ax.set_box_aspect(high-low)
    ax.view_init(elev=25,azim=-58)
    ax = fig.add_subplot(2,2,col+3)
    for name,color in [('a','#1687cd'),('b','#cd5ab4')]:
        cloud=points[name]
        cloud=cloud[(cloud[:,2]>=-.25)&(cloud[:,2]<=.75)]
        ax.scatter(cloud[:,0],cloud[:,1],c=color,s=.7,alpha=.65,rasterized=True,label='Odin '+name.upper())
    ax.set(title=title+'：水平切片',xlabel='X (m)',ylabel='Y (m)')
    ax.set_xlim(low[0],high[0]);ax.set_ylim(low[1],high[1]);ax.set_aspect('equal',adjustable='box')
    ax.grid(alpha=.2);ax.legend(fontsize=9,loc='upper right')
    q=report['modes'][mode]['best_candidate']['quality']
    stats.append(f'{title}：10 cm 内双向匹配比例 {q["b_to_a_overlap_fraction"]:.0%} / {q["a_to_b_overlap_fraction"]:.0%}；'
                 f'B→A 匹配点距离 RMSE {q["b_to_a_inlier_rmse_m"]*100:.1f} cm')
fig.text(.06,.074,stats[0],fontsize=10,color='#435568')
fig.text(.06,.050,stats[1],fontsize=10,color='#435568')
fig.text(.06,.027,'水平切片：本地 Z ∈ [-0.25, 0.75] m。两种轨迹分别配准；匹配指标不是对真值的地图精度。',fontsize=10,color='#435568')
fig.subplots_adjust(top=.90,bottom=.14,left=.06,right=.96,hspace=.2,wspace=.22)
fig.savefig(OUT / 'fusion_comparison.png',dpi=170)
fig.savefig(OUT / 'fusion_comparison.pdf')
plt.close(fig)

# No network libraries: raw WebGL draws the real voxelized point positions.
payload={}
for mode, devices in maps.items():
    payload[mode]={}
    for name,points in devices.items():
        payload[mode][name]={'count':len(points),'base64':base64.b64encode(points.astype('<f4').tobytes()).decode()}
html=r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>双 Odin1 点云配准预览</title><style>
body{margin:0;background:#0e1729;color:#edf3ff;font:15px system-ui,sans-serif}header{padding:20px 24px;background:#14223a}h1{margin:0 0 10px;font-size:25px}p{margin:7px 0;color:#c1cde0}.controls{display:flex;gap:18px;align-items:center;flex-wrap:wrap;margin-top:14px}select,button{background:#203653;color:#fff;border:1px solid #527094;border-radius:5px;padding:7px 12px;font:inherit}label{cursor:pointer}canvas{width:100%;height:calc(100vh - 235px);display:block;touch-action:none}#info{padding:12px 24px;font-size:14px}.blue{color:#36b5ff}.pink{color:#ff91d5}
</style><header><h1>双 Odin1 点云 · 离线候选配准</h1>
<p>每台先独立累积，再用几何 ICP 估计一个固定变换。尚未接入 Kimera-Multi 跨机回环或分布式优化。</p>
<p>左键拖动旋转，右键拖动平移，滚轮缩放。SDK 是参考里程计；匹配距离不是地图真值误差。</p>
<div class="controls"><select id="mode"><option value="vio">Kimera VIO 轨迹累积</option><option value="sdk">Odin SDK 参考轨迹累积</option></select>
<label class="blue"><input id="a" type="checkbox" checked>Odin A</label><label class="pink"><input id="b" type="checkbox" checked>Odin B</label>
<button id="reset">重置视角</button><button id="top">俯视</button><label><input id="slice" type="checkbox">水平切片 Z ∈ [-0.25, 0.75] m</label>
<label>点大小 <input id="size" type="range" min="1" max="5" step=".5" value="2"></label></div></header>
<canvas id="canvas"></canvas><div id="info"></div><script>
const DATA=__DATA__, REPORT=__REPORT__;
const canvas=document.getElementById('canvas'),gl=canvas.getContext('webgl',{antialias:true,preserveDrawingBuffer:true});
if(!gl)throw Error('浏览器不支持 WebGL');
const vertex=`attribute vec3 p;uniform vec3 center;uniform vec2 angle;uniform vec2 pan;uniform float scale;uniform float aspect;uniform float size;varying float z;void main(){vec3 q=p-center;float c=cos(angle.x),s=sin(angle.x);float x=c*q.x-s*q.y,y=s*q.x+c*q.y;float v=sin(angle.y)*y+cos(angle.y)*q.z;float d=cos(angle.y)*y-sin(angle.y)*q.z;gl_Position=vec4((x*scale+pan.x)/aspect,v*scale+pan.y,d*0.025,1.);gl_PointSize=size;z=p.z;}`;
const fragment=`precision mediump float;uniform vec3 color;uniform bool slice;varying float z;void main(){if(slice&&(z<-.25||z>.75))discard;gl_FragColor=vec4(color,1.);}`;
function shader(type,src){let s=gl.createShader(type);gl.shaderSource(s,src);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw Error(gl.getShaderInfoLog(s));return s}
const prog=gl.createProgram();gl.attachShader(prog,shader(gl.VERTEX_SHADER,vertex));gl.attachShader(prog,shader(gl.FRAGMENT_SHADER,fragment));gl.linkProgram(prog);gl.useProgram(prog);
const uniforms={};for(const key of ['center','angle','pan','scale','aspect','size','color','slice'])uniforms[key]=gl.getUniformLocation(prog,key);
const locationP=gl.getAttribLocation(prog,'p');gl.enableVertexAttribArray(locationP);
const buffers={};let bounds={};for(const [mode,devices] of Object.entries(DATA)){buffers[mode]={};let lo=[Infinity,Infinity,Infinity],hi=[-Infinity,-Infinity,-Infinity];for(const [name,item] of Object.entries(devices)){let bytes=Uint8Array.from(atob(item.base64),x=>x.charCodeAt(0));let points=new Float32Array(bytes.buffer);let buffer=gl.createBuffer();gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,points,gl.STATIC_DRAW);buffers[mode][name]={buffer,count:item.count};for(let i=0;i<points.length;i++){let k=i%3;lo[k]=Math.min(lo[k],points[i]);hi[k]=Math.max(hi[k],points[i])}}bounds[mode]={center:lo.map((x,i)=>(x+hi[i])/2),extent:Math.max(hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2])}}
let angle=[-.85,.55],pan=[0,0],zoom=1;const modeEl=document.getElementById('mode');
function draw(){let mode=modeEl.value,b=bounds[mode];let width=Math.floor(canvas.clientWidth*devicePixelRatio),height=Math.floor(canvas.clientHeight*devicePixelRatio);if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}gl.viewport(0,0,width,height);gl.clearColor(.055,.09,.16,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);gl.enable(gl.DEPTH_TEST);gl.uniform3fv(uniforms.center,b.center);gl.uniform2fv(uniforms.angle,angle);gl.uniform2fv(uniforms.pan,pan);gl.uniform1f(uniforms.scale,zoom*1.65/b.extent);gl.uniform1f(uniforms.aspect,width/height);gl.uniform1f(uniforms.size,Number(document.getElementById('size').value)*devicePixelRatio);gl.uniform1i(uniforms.slice,document.getElementById('slice').checked);for(const name of ['a','b']){if(!document.getElementById(name).checked)continue;let entry=buffers[mode][name];gl.bindBuffer(gl.ARRAY_BUFFER,entry.buffer);gl.vertexAttribPointer(locationP,3,gl.FLOAT,false,0,0);gl.uniform3fv(uniforms.color,name==='a'?[.12,.57,.90]:[.92,.39,.73]);gl.drawArrays(gl.POINTS,0,entry.count)}let q=REPORT.modes[mode].best_candidate.quality;document.getElementById('info').textContent=`A ${buffers[mode].a.count.toLocaleString()} 点，B ${buffers[mode].b.count.toLocaleString()} 点｜10 cm 内匹配比例 B→A ${(q.b_to_a_overlap_fraction*100).toFixed(1)}%，A→B ${(q.a_to_b_overlap_fraction*100).toFixed(1)}%｜匹配点 RMSE ${(q.b_to_a_inlier_rmse_m*100).toFixed(1)} cm（拟合诊断）`}
let dragging=null;canvas.oncontextmenu=e=>e.preventDefault();canvas.onpointerdown=e=>{dragging={x:e.clientX,y:e.clientY,button:e.button};canvas.setPointerCapture(e.pointerId)};canvas.onpointerup=()=>dragging=null;canvas.onpointermove=e=>{if(!dragging)return;let dx=e.clientX-dragging.x,dy=e.clientY-dragging.y;if(dragging.button===2){pan[0]+=dx/canvas.clientHeight*2;pan[1]-=dy/canvas.clientHeight*2}else{angle[0]+=dx*.008;angle[1]=Math.max(-1.57,Math.min(1.57,angle[1]-dy*.008))}dragging.x=e.clientX;dragging.y=e.clientY;draw()};canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.2,Math.min(20,zoom*Math.exp(-e.deltaY*.001)));draw()},{passive:false});
document.getElementById('reset').onclick=()=>{angle=[-.85,.55];pan=[0,0];zoom=1;draw()};document.getElementById('top').onclick=()=>{angle=[0,Math.PI/2];pan=[0,0];draw()};for(const id of ['mode','a','b','slice','size'])document.getElementById(id).oninput=draw;window.onresize=draw;draw();
</script></html>'''
html=html.replace('__DATA__',json.dumps(payload,separators=(',',':'))).replace('__REPORT__',json.dumps(report,separators=(',',':')))
(OUT / 'viewer.html').write_text(html)
print(OUT / 'fusion_comparison.png')
print(OUT / 'viewer.html')
