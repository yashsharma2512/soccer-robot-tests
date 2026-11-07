const fs=require('fs'),vm=require('vm'),assert=require('assert');
const page=fs.readFileSync(__dirname+'/../ESP32SoccerAP/PlayPage.h','utf8');
const script=page.match(/<script>([\s\S]*?)<\/script>/)[1];
const benchTest=fs.readFileSync(__dirname+'/bench_page.cjs','utf8');
const data=vm.runInNewContext('('+benchTest.match(/const data=([\s\S]*?);\nlet lastBlob/)[1]+')');
Object.assign(data,{playState:'FOLLOW',playing:true,canResume:true});
Object.assign(data.motor,{enabled:true,active:true,watchdog:false});
const elements=new Map(),requests=[],calButtons=Array.from({length:4},()=>({disabled:false}));
function element(id){if(!elements.has(id))elements.set(id,{textContent:'',innerHTML:'',className:'',disabled:false});return elements.get(id);}
let lastBlob;
const context=vm.createContext({document:{getElementById:element,querySelectorAll:()=>calButtons,createElement:()=>({click(){}})},Date,AbortController,URLSearchParams,Blob:class{constructor(parts){lastBlob=parts.join('');}},URL:{createObjectURL(){return 'blob:test';},revokeObjectURL(){}},setTimeout(){return 1;},clearTimeout(){},setInterval(){},fetch:async(url,options)=>{requests.push({url,options});return {ok:true,json:async()=>data,text:async()=> 'Queued'};}});
(async()=>{
 vm.runInContext(script,context);await new Promise(setImmediate);
 assert.equal(element('playState').textContent,'FOLLOW');
 assert(element('resume').disabled);assert(calButtons.every(b=>!b.disabled)); // Calibrate can stop active play.
 data.playing=false;data.motor.active=false;vm.runInContext('render(latest)',context);assert(!element('resume').disabled);
 data.busy=true;vm.runInContext('render(latest)',context);assert(calButtons.every(b=>b.disabled));
 data.busy=false;data.canResume=false;vm.runInContext('render(latest)',context);assert(element('resume').disabled);
 data.motor.watchdog=true;vm.runInContext('render(latest)',context);assert.equal(element('motorState').textContent,'WATCHDOG STOP');
 vm.runInContext('lastSeen=0;controls()',context);assert(element('resume').disabled && calButtons.every(b=>b.disabled));
 await vm.runInContext("act('fullcal')",context);let r=requests.at(-1);
 assert.equal(r.options.method,'POST');assert.equal(r.options.headers['X-Bench-Key'],'__KEY__');assert.equal(r.options.body.get('action'),'fullcal');
 await vm.runInContext("act('stop')",context);assert.equal(requests.at(-1).options.body.get('action'),'stop');
 vm.runInContext('download()',context);const rows=lastBlob.split('\n');assert.equal(rows[0].split(',').length,46);assert.equal(rows[1].split(',').length,46);
 console.log('PASS: full-play page renders, calibration during play, Resume health/busy/offline gating, watchdog status, authenticated actions, 46-column CSV.');
})().catch(e=>{console.error(e);process.exitCode=1;});
