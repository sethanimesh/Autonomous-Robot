/** Camera lifecycle checks in an isolated DOM/media harness, without camera access. */
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {test} from 'node:test';
const source=readFileSync(new URL('../robot/jetson/perception/enrollment_console.py',import.meta.url),'utf8');
const code=source.split('// PHONE_ENROLLMENT_BEGIN\n')[1].split('// PHONE_ENROLLMENT_END')[0];
function deferred(){let resolve,reject;let promise=new Promise((a,b)=>{resolve=a;reject=b});return {promise,resolve,reject}}
function harness(){
  let elements=new Map(),timers=new Map(),nextTimer=0,posts=[],streams=[];
  function element(id){if(!elements.has(id))elements.set(id,{id,value:id==='enrollmentSource'?'phone':'',checked:true,disabled:false,hidden:false,dataset:{},open:true,videoWidth:640,videoHeight:480,
    pause(){},async play(){},removeAttribute(k){delete this[k]},getAttribute(k){return this[k]??null},scrollIntoView(){},files:[]});return elements.get(id)}
  let storage=new Map([['echoraEnrollmentSession','session-1']]);
  let ctx={console,AbortController,Image:class{},URL,
    sessionStorage:{getItem:k=>storage.get(k)||null,setItem:(k,v)=>storage.set(k,v),removeItem:k=>storage.delete(k)},
    latestStatus:{enrolling:true,enrollment_session_id:'session-1',enrollment_source:'phone',ready:false,mission:{},manual_drive:{}},
    window:{isSecureContext:true},document:{hidden:false,getElementById:element,createElement(tag){assert.equal(tag,'canvas');return {width:0,height:0,getContext(){return {drawImage(...args){ctx.draw=args}}},toBlob(callback){callback({type:'image/jpeg'})}}}},
    navigator:{mediaDevices:{async getUserMedia(options){assert.equal(options.audio,false);let track={stopped:false,stop(){this.stopped=true},getSettings:()=>({facingMode:options.video.facingMode.ideal}),addEventListener(){}};let stream={getTracks:()=>[track],getVideoTracks:()=>[track]};streams.push(stream);return stream}}},
    setTimeout(fn,ms){let id=++nextTimer;timers.set(id,{fn,ms});return id},clearTimeout:id=>timers.delete(id),
    setText(id,text){element(id).textContent=text},alert(message){ctx.alerts.push(message)},alerts:[],
    async refresh(){ctx.refreshes++;},refreshes:0,
    async post(...args){posts.push(args);return {added:true,ready:false,message:'Captured',guidance:'Turn slightly left'}},
    updateFamilyId:null,addingFamilyMember:false};
  vm.createContext(ctx);vm.runInContext(code,ctx);
  return {ctx,element,timers,posts,streams,run:s=>vm.runInContext(s,ctx)};
}
test('phone permission opens video only, and pause releases every camera track',async()=>{
  let h=harness();await h.ctx.resumePhoneCapture();assert.equal(h.streams.length,1);assert.equal(h.element('phonePreview').dataset.facing,'user');
  assert.equal(h.timers.size,1);h.ctx.pausePhoneCapture();assert(h.streams[0].getTracks()[0].stopped);assert.equal(h.element('phonePreview').srcObject,null);assert.equal(h.timers.size,0);
});
test('permission denied leaves enrollment usable for photo upload and retry',async()=>{
  let h=harness();h.ctx.navigator.mediaDevices.getUserMedia=async()=>{let e=Error();e.name='NotAllowedError';throw e};
  await h.ctx.resumePhoneCapture();h.ctx.updateEnrollmentControls();assert.match(h.element('phoneMessage').textContent,/permission/);
  assert.equal(h.element('upload').disabled,false);assert.equal(h.element('resumePhone').disabled,false);assert.equal(h.run('phone.requesting'),false);
});
test('late permission response after closing cannot reactivate capture',async()=>{
  let h=harness(),permission=deferred(),stopped=false;
  h.ctx.navigator.mediaDevices.getUserMedia=()=>permission.promise;let pending=h.ctx.resumePhoneCapture();h.ctx.pausePhoneCapture();
  permission.resolve({getTracks:()=>[{stop(){stopped=true}}]});await pending;
  assert(stopped);assert.equal(h.run('phone.stream'),null);assert.equal(h.timers.size,0);
});
test('capture binds requests to the session and never overlaps frame requests',async()=>{
  let h=harness();await h.ctx.resumePhoneCapture();let response=deferred();h.ctx.post=async(...args)=>{h.posts.push(args);return response.promise};
  let first=h.run('capturePhoneFrame(phone.generation,enrollmentToken)');await Promise.resolve();await Promise.resolve();
  await h.run('capturePhoneFrame(phone.generation,enrollmentToken)');assert.equal(h.posts.length,1);
  assert.equal(h.posts[0][0],'/api/enrollment/frame');assert.equal(h.posts[0][3].headers['X-Echora-Enrollment'],'session-1');
  response.resolve({added:true,ready:true,message:'Done',guidance:'Ready to finish'});await first;
  assert.equal(h.run('phone.ready'),true);assert.match(h.element('phoneMessage').textContent,/Finish enrollment/);
  assert.equal(h.ctx.draw[0],h.element('phonePreview')); // CSS mirroring is not applied to saved pixels.
});
test('camera switch closes the old stream before acquiring another',async()=>{
  let h=harness();await h.ctx.resumePhoneCapture();await h.ctx.switchPhoneCamera();assert(h.streams[0].getTracks()[0].stopped);
  assert.equal(h.streams.length,2);assert.equal(h.element('phonePreview').dataset.facing,'environment');
});
test('backgrounding, changed session and lost connection stop acquisition',async()=>{
  let h=harness();await h.ctx.resumePhoneCapture();h.ctx.document.hidden=true;await h.run('capturePhoneFrame(phone.generation,enrollmentToken)');
  assert.equal(h.run('phone.stream'),null);assert.equal(h.posts.length,0);
  h.ctx.document.hidden=false;await h.ctx.resumePhoneCapture();h.ctx.latestStatus.enrollment_session_id='new-session';h.ctx.updateEnrollmentControls();
  assert.equal(h.run('phone.stream'),null);assert.equal(h.element('finish').disabled,true);
  h.ctx.latestStatus.enrollment_session_id='session-1';await h.ctx.resumePhoneCapture();h.ctx.post=async()=>{throw Error('Offline')};
  await h.run('capturePhoneFrame(phone.generation,enrollmentToken)');assert.equal(h.run('phone.stream'),null);assert.match(h.element('phoneMessage').textContent,/connection interrupted/);
});
test('HTTP phone access offers HTTPS setup and uploads without calling camera API',async()=>{
  let h=harness();h.ctx.window.isSecureContext=false;h.ctx.updateEnrollmentControls();assert.equal(h.element('secureCameraHelp').hidden,false);
  assert.equal(h.element('upload').disabled,false);await h.ctx.resumePhoneCapture();assert.equal(h.streams.length,0);
});
test('robot enrollment does not enable phone capture controls',()=>{
  let h=harness();h.ctx.latestStatus.enrollment_source='robot';h.ctx.latestStatus.camera_ready=true;h.ctx.updateEnrollmentControls();
  assert.equal(h.element('phoneCapture').hidden,true);assert.equal(h.element('resumePhone').disabled,true);assert.equal(h.element('robotEnrollmentPreview').src,'/stream?enrollment=1');
});
