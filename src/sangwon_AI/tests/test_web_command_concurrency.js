// Execute the actual page handler with two independently delayed HTTP replies.
const fs = require('fs');
const vm = require('vm');
const assert = require('assert/strict');
const path = require('path');
const page = fs.readFileSync(path.join(__dirname,'../../drone_mission/drone_mission/web_page.py'),'utf8');
const handler = page.slice(page.indexOf('async function command(action)'),page.indexOf("try{pending=JSON.parse"));
async function run(rejectOld) {
  const requests=[];
  const context=vm.createContext({requests,crypto:{randomUUID:()=>require('crypto').randomBytes(16).toString('hex')},sessionStorage:{setItem(){},removeItem(){}}});
  vm.runInContext(`let busy=false,landBusy=false,commandGeneration=0,pending=null,selected=null,dirty=false;
    const notice={},by=()=>({}),definition=()=>({trial_case:'hover'});
    const lastStatus={assignment:{drone_id:'6',anchor_layout_id:'test'},telemetry:{operator_session_id:'s'}};
    const refreshCatalog=async()=>{};
    function post(path,body){return new Promise((resolve,reject)=>requests.push({body,resolve,reject}));}
    ${handler}`,context);
  const start=vm.runInContext("command('start')",context);
  assert.equal(requests.length,1);
  const land=vm.runInContext("command('takeoff_land')",context);
  assert.equal(requests.length,2,'LAND was blocked by pending START HTTP');
  assert.equal(requests[1].body.action,'takeoff_land');
  if(rejectOld)requests[0].reject(Object.assign(Error('superseded'),{responded:true}));
  else requests[0].resolve({mission_db_id:1,dispatch_cancelled:true});
  await start;
  assert.equal(vm.runInContext('pending.action',context),'takeoff_land','late START erased LAND receipt');
  assert.equal(vm.runInContext('landBusy',context),true);
  requests[1].resolve({});await land;
  assert.equal(vm.runInContext('pending',context),null);
  assert.equal(vm.runInContext('landBusy',context),false);
}
(async()=>{await run(false);await run(true);console.log('2 browser command concurrency cases passed');})().catch(e=>{console.error(e);process.exitCode=1;});
