// Execute the actual embedded page script with a small DOM/network double.
// Checks rendering, freshness gating, action authentication and CSV structure.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(__dirname+'/../ESP32BenchDashboard/BenchPage.h','utf8');
const script=source.match(/<script>([\s\S]*?)<\/script>/)[1];
const elements=new Map(),buttons=[],requests=[];
function element(id){if(!elements.has(id))elements.set(id,{textContent:'',innerHTML:'',className:'',checked:false,value:id==='pwm'?'80':id==='duration'?'400':'',disabled:false,dataset:{},append(b){buttons.push(b);},click(){}});return elements.get(id);}
const data={ms:12345,busy:false,nvs:true,mux:true,mpu:{ok:true,id:112,address:104,error:'OK',calibrating:false,progress:500,loaded:true,yaw:1.2,rate:0.1,bias:131,age:2,samples:800,failedTransactions:2,recovered:1,register:58,wireStatus:2,bytes:0,raw:[0,0,16384,0,0,0,131]},ir:{ok:true,direction:5,strength:90,age:10,good:120,failures:0,wireStatus:0,bytes:6,detectors:[0,10,90,15,0]},lineCalibrated:true,lineCapturing:false,lineSaving:false,apds:Array.from({length:4},(_,i)=>({channel:i,side:1<<i,id:158,ok:true,age:10,clear:100,rgb:[50,80,30],white:false,threshold:130,release:115,calibrated:true,progress:32,errors:0,fault:0})),events:['12000 ms: ready'],motor:{ready:true,armed:false,active:false,wheel:[0,0,0,0]}};
let lastBlob;
const context=vm.createContext({document:{getElementById:element,querySelector:s=>element(s.slice(1)),createElement:()=>({dataset:{},disabled:false,click(){}}),querySelectorAll:s=>s==='[data-motor]'?buttons.filter(b=>b.dataset.motor):[]},Date,AbortController,URLSearchParams,Blob:class{constructor(parts){lastBlob=parts.join('');}},URL:{createObjectURL(){return 'blob:test';},revokeObjectURL(){}},setTimeout(){return 1;},clearTimeout(){},setInterval(){},fetch:async(url,options)=>{requests.push({url,options});return {ok:true,json:async()=>data,text:async()=> 'OK'};}});
(async()=>{
 vm.runInContext(script,context);await new Promise(setImmediate);
 assert(element('mpu').innerHTML.includes('0x70'));assert(element('ir').innerHTML.includes('Direction 5'));
 assert(element('apds').innerHTML.includes('CH3'));assert(buttons.length===14);
 assert(buttons.every(b=>b.disabled)); // Disarmed prevents movement.
 data.motor.armed=true;vm.runInContext('render(latest)',context);assert(buttons.every(b=>!b.disabled));
 data.busy=true;vm.runInContext('render(latest)',context);assert(buttons.every(b=>b.disabled));
 data.busy=false;data.apds[0].ok=false;vm.runInContext('render(latest)',context);assert(element('apds').innerHTML.includes('unknown'));
 vm.runInContext('lastSeen=0;controls()',context);assert(buttons.every(b=>b.disabled));assert(element('connection').textContent.includes('OFFLINE'));
 await vm.runInContext("act('pulse',{target:'FL',pwm:'80',ms:'400',reverse:'0'})",context);
 const r=requests.at(-1);assert.equal(r.options.method,'POST');assert.equal(r.options.headers['X-Bench-Key'],'__KEY__');assert(r.options.body.toString().includes('target=FL'));
 vm.runInContext('download()',context);const rows=lastBlob.split('\n');assert(rows.length>1);assert.equal(rows[0].split(',').length,rows[1].split(',').length);assert.equal(rows[0].split(',').length,44);
 console.log('PASS: embedded JS rendering, all 14 motor buttons, disarm/busy/offline gating, authenticated POST, 44-column CSV.');
})().catch(e=>{console.error(e);process.exitCode=1;});
