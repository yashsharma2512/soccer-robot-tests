#pragma once
#include <Arduino.h>
const char PLAY_PAGE[] PROGMEM=R"PLAY(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Soccer play</title><style>
:root{color-scheme:dark;font:15px system-ui;background:#101820;color:#e8edf1}*{box-sizing:border-box}
body{max-width:1180px;margin:auto;padding:20px}h1{font-size:30px;margin:0}h2{font-size:19px;margin:0 0 12px}
header{display:flex;justify-content:space-between;gap:20px;align-items:center;margin-bottom:18px}
p{line-height:1.5;color:#b9c7d2}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:14px}
section{background:#192632;border:1px solid #314454;border-radius:14px;padding:18px}.wide{grid-column:1/-1}
button,input,select{font:inherit;border:1px solid #4e677b;border-radius:7px;padding:9px;background:#24394b;color:#eef6ff}
button{cursor:pointer}button:disabled{opacity:.35;cursor:default}button:hover:enabled{background:#35536b}
.stop{background:#b52739;font-weight:800;font-size:20px;min-width:120px}.stop:hover:enabled{background:#db3548}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:10px 0}.pill{border-radius:20px;padding:5px 10px;background:#394b5a}
.good{color:#8ee3b6}.bad{color:#ff9c91}.muted{color:#9daebc}.stale{opacity:.55}code,pre{font:12px ui-monospace,monospace}
pre{white-space:pre-wrap;line-height:1.6;margin:8px 0}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:9px 6px;border-bottom:1px solid #304354;font-size:14px}
.scroll{overflow-x:auto}.bars{display:flex;gap:8px;height:85px;align-items:end;margin:16px 0}.bar{flex:1;background:#58b6d2;min-height:2px;position:relative}.bar span{position:absolute;top:-20px;width:100%;text-align:center;font-size:12px}
#notice{min-height:24px;color:#f9cf81}label{display:flex;gap:8px;align-items:center}input[type=number]{width:85px}
small{color:#a7bac9}#connection{white-space:nowrap}@media(max-width:600px){body{padding:12px}header{align-items:start}.grid{grid-template-columns:1fr}h1{font-size:25px}}
</style>
<header><div><h1>Soccer play</h1><small>ESP32 · autonomous soccer · local AP</small></div><span id="connection" class="pill">Connecting…</span></header>
<div class="row"><button class="stop" onclick="act('stop')">STOP</button><span id="summary">Waiting for device</span></div>
<p>Normal play follows the ball, repeats its search when the ball is lost, and escapes white lines. Calibration stops driving until you press Resume. Valid saved calibration enables automatic startup.</p>
<div id="notice" role="status" aria-live="polite"></div>
<div class="grid">
<section class="wide"><h2>Play control</h2><div class="row"><button id="resume" onclick="act('resume')">Resume play</button><button data-cal onclick="act('fullcal')">Calibrate gyro then GREEN + save</button><span id="motorState" class="pill">STOPPED</span></div>
<p id="playState">Waiting for control task...</p><small>FL 25/26 · FR 16/17 · BL 32/33 (sign -1) · BR 18/19. Applied logical PWM: <span id="wheelValues">0 / 0 / 0 / 0</span>. No encoder feedback. A 100 ms control watchdog stops stale motor commands; it does not stop play merely because a phone disconnects.</small></section>
<section><h2>MPU</h2><div id="mpu"></div><div class="row"><button data-cal onclick="act('gyro')">Calibrate gyro + save</button></div><small>Keep the robot still. Saved bias is shared with ESP32SoccerFull / ESP32SoccerAP. Yaw is relative and drifts; it is not a compass heading.</small></section>
<section><h2>HiTechnic IR</h2><div id="ir"></div><div class="bars" id="bars"></div><small>AC / pulsed IR · address 0x08 · direction 5 is centre; 0 means no ball. A valid sensor read can still report no ball.</small></section>
<section><h2>Calibration + system</h2><div id="system"></div><div class="row"><button data-cal onclick="act('green')">Capture GREEN + save</button><button data-cal onclick="act('rescan')">Rescan / reload</button></div><div class="row"><button onclick="act('marker')">Mark battery switch</button><button id="csv" onclick="download()">Download CSV</button></div><small>Green capture: 3 s to position all four sensors on the field, then 32 samples each. Test white detection under real lighting afterward. Calibration remains stopped until Resume. No NeoPixels.</small></section>
<section class="wide"><h2>APDS boundary sensors</h2><div class="scroll"><table><thead><tr><th>Channel / side</th><th>ID / status</th><th>Clear</th><th>R / G / B</th><th>On / release</th><th>Line</th><th>Age / errors</th></tr></thead><tbody id="apds"></tbody></table></div><small>Side mapping is an example: CH0 front, CH1 right, CH2 rear, CH3 left. Edit boundarycfg::SIDES in BenchSensors.h to match your robot. Gray readings are stale. Calibrate again after changing sensor gain, height, or lighting.</small></section>
<section class="wide"><h2>Events</h2><pre id="events">Waiting…</pre><small>Press Mark battery switch immediately before switching the battery on, then compare MPU failures and IR/APDS freshness. Counter resets on reboot; CSV holds the last 2,000 webpage samples.</small></section>
</div>
<script>
'use strict';
const key='__KEY__', $=id=>document.getElementById(id);
let latest=null,lastSeen=0,samples=[],inFlight=false;
const hex=n=>'0x'+Number(n).toString(16).toUpperCase();
const state=(ok,text)=>`<span class="${ok?'good':'bad'}">${text}</span>`;
const sides={1:'front',2:'right',4:'rear',8:'left',3:'front-right',6:'rear-right',12:'rear-left',9:'front-left'};
const faults=['OK','WAITING','MISSING','READ ERROR','SATURATED','CONFIG ERROR'];
function controls(){
 const online=Date.now()-lastSeen<1800 && latest?.motor;
 document.querySelectorAll('[data-cal]').forEach(b=>b.disabled=!(online && !latest.busy && !inFlight));
 $('resume').disabled=!(online && latest.canResume && !latest.playing && !inFlight);
 $('connection').textContent=online?'LIVE':'OFFLINE / STALE';$('connection').className='pill '+(online?'good':'bad');
}
async function act(action,extra={}){
 if(inFlight && action!=='stop')return;
 inFlight=true;controls();
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),2500);
 try{
  const r=await fetch('/api/action',{method:'POST',headers:{'X-Bench-Key':key,'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({action,...extra}),signal:controller.signal});
  $('notice').textContent=(r.ok?'':'Rejected: ')+await r.text();
 }catch(e){$('notice').textContent='Connection lost. Web STOP needs a connection; use the motor power switch to stop immediately.';}
 finally{clearTimeout(timer);inFlight=false;controls();}
}
function render(d){
 latest=d;lastSeen=Date.now();const m=d.mpu,ir=d.ir;
 $('summary').textContent=`Device ${Math.round(d.ms/1000)} s · ${d.busy?'calibration / startup busy':d.playing?'play enabled':'stopped'} · main I²C SDA 21 / SCL 22`;
 $('motorState').textContent=d.motor.watchdog?'WATCHDOG STOP':d.playing?'PLAY ENABLED':'STOPPED';
 $('playState').textContent=d.playState+(d.busy?' · calibration / startup busy':'')+(d.canResume&&!d.playing?' · ready to resume':'');
 $('wheelValues').textContent=d.motor.wheel.join(' / ');
 $('mpu').innerHTML=state(m.ok,m.error)+(m.calibrating?` · calibrating ${m.progress}/500`:'')+`<pre>Address ${hex(m.address)} · WHO_AM_I ${hex(m.id)}\nYaw ${m.yaw}° · rate ${m.rate}°/s\nBias ${m.bias} raw · sample age ${m.age} ms\nSamples ${m.samples} · failed transfers ${m.failedTransactions}\nRetry recoveries ${m.recovered}\nLast failed transfer: reg ${hex(m.register)}, status ${m.wireStatus}, bytes ${m.bytes}\nRaw ax/ay/az, temp, gx/gy/gz:\n${m.raw.join(' / ')}\nSaved bias loaded: ${m.loaded?'yes':'no'}</pre>`;
 $('ir').className=ir.ok?'':'stale';
 $('ir').innerHTML=state(ir.ok,ir.ok?'READ OK':'READ FAILED / STALE')+`<pre>Direction ${ir.direction} · strength ${ir.strength}\nAge ${ir.age} ms · good ${ir.good} · failures ${ir.failures}\nWire status ${ir.wireStatus} · bytes ${ir.bytes}/6</pre>`;
 $('bars').className='bars'+(ir.ok?'':' stale');
 $('bars').innerHTML=ir.detectors.map(n=>`<div class="bar" style="height:${Math.max(2,n/255*65)}px"><span>${n}</span></div>`).join('');
 $('system').innerHTML=`<pre>NVS ${d.nvs?'ready':'FAILED'} · mux ${d.mux?'isolated / OK':'FAULT'}\nLine calibration ${d.lineCalibrated?'ready':'required / invalid'}\nCapture ${d.lineCapturing?'in progress':d.lineSaving?'saving':'idle'}\nMotor PWM ${d.motor.ready?'ready':'FAILED'}</pre>`;
 $('apds').innerHTML=d.apds.map(a=>`<tr class="${a.ok?'':'stale'}"><td>CH${a.channel} / ${sides[a.side]||a.side}</td><td>${hex(a.id)} / ${state(a.ok,a.ok?'fresh':(a.fault===0?'STALE':faults[a.fault]||'STALE'))}</td><td>${a.clear}</td><td>${a.rgb.join(' / ')}</td><td>${a.calibrated?a.threshold+' / '+a.release:'uncalibrated'}${d.lineCapturing?' ('+a.progress+'/32)':''}</td><td>${a.ok&&a.calibrated?state(!a.white,a.white?'WHITE':'field'):'unknown'}</td><td>${a.age} ms / ${a.errors}</td></tr>`).join('');
 $('events').textContent=d.events.join('\n')||'No events';
 samples.push([d.ms,d.playState,d.playing,m.ok,m.id,m.error,m.yaw,m.rate,m.failedTransactions,m.recovered,ir.ok,ir.direction,ir.strength,ir.failures,...d.apds.flatMap(a=>[a.clear,...a.rgb,a.white,a.ok,a.errors]),...d.motor.wheel]);if(samples.length>2000)samples.shift();
 controls();
}
async function poll(){
 const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),1500);
 try{const r=await fetch('/api/status',{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error();const d=await r.json();if(d.mpu&&d.motor)render(d);}catch(e){controls();}
 finally{clearTimeout(timer);setTimeout(poll,300);}
}
function download(){
 const columns=['ms','play_state','play_enabled','mpu_ok','mpu_id','mpu_error','yaw','rate','mpu_failed_transfers','mpu_recovered','ir_ok','ir_direction','ir_strength','ir_failures'];
 for(let i=0;i<4;i++)columns.push(...['clear','red','green','blue','white','fresh','errors'].map(x=>'apds'+i+'_'+x));columns.push('FL','FR','BL','BR');
 const csv=[columns,...samples].map(row=>row.map(v=>'"'+String(v).replaceAll('"','""')+'"').join(',')).join('\n');
 const url=URL.createObjectURL(new Blob([csv],{type:'text/csv'})),a=document.createElement('a');a.href=url;a.download='soccer-bench.csv';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
setInterval(controls,400);controls();poll();
</script></html>)PLAY";
