const fs=require('node:fs'), vm=require('node:vm'), assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../src/blupe_controller/operator_page.py'),'utf8');
const code=source.match(/aq\.disabled=([^;]+);/)[1];
const disabled=(extra={}, cloudExtra={})=>{
  const s={mode:'readonly',at_zero:true,at_home:false,home_captured:true,zero_captured:true,...extra};
  const cloud={connected:true,auto_queue:false,...cloudExtra};
  return vm.runInNewContext(code,{s,cloud,active:s.mode==='active',busy:cloud.session_active||cloud.queue_ready||s.manual_motion});
};
assert.equal(disabled(),false);
assert.equal(disabled({mode:'active'}),false);
assert.equal(disabled({mode:'active',at_home:true,at_zero:false}),false);
for(const s of [{at_zero:false},{mode:'fault'},{home_captured:false},{zero_captured:false},{manual_motion:true},{calibration_ready:false}])assert.equal(disabled(s),true);
for(const c of [{connected:false},{session_active:true},{returning_home:true}])assert.equal(disabled({},c),true);
assert.equal(disabled({manual_motion:true},{auto_queue:true}),false);
console.log('Auto-queue can be enabled at Zero or active Home; invalid states stay blocked.');
