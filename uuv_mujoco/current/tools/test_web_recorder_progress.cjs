const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../gui/web_static/recorder.js'), 'utf8');
function fixture() {
  const elements = Object.fromEntries(['recTargetSeconds','recProgress','recProgressPercent','recProgressTime','recProgressHint'].map(id=>[id,{value: id === 'recTargetSeconds' ? 30 : 0}]));
  const ctx = vm.createContext({el:id=>elements[id], pending:false});
  vm.runInContext(source.slice(source.indexOf('  let lastRecorderStatus'),source.indexOf("  el('recTargetSeconds').addEventListener")),ctx);
  return {elements, render:ctx.renderProgress};
}
test('progress uses measured simulation duration and caps at 100 without stopping',()=>{
  const {elements:e,render}=fixture();
  render({active:true,online:true,duration_s:15});
  assert.equal(e.recProgress.value,50);
  assert.equal(e.recTargetSeconds.disabled,true);
  render({active:true,online:true,duration_s:40});
  assert.equal(e.recProgress.value,100);
  assert.match(e.recProgressHint.textContent,/자동 종료되지 않습니다/);
});
test('saved episode persists as approximate duration and new episode resets',()=>{
  const {elements:e,render}=fixture();
  render({active:false,online:true,last_result:{path:'episode',frames:155}});
  assert.match(e.recProgressTime.textContent,/최근 저장 약 15.4/);
  render({active:true,online:true,episode_index:1,duration_s:0});
  assert.equal(e.recProgress.value,0);
  render({active:false,last_result:{termination_reason:'discarded'}});
  assert.equal(e.recProgress.value,0);
});
test('stale recorder shows last observation warning instead of counting wall time',()=>{
  const {elements:e,render}=fixture();
  render({active:true,online:false,duration_s:12});
  assert.equal(e.recProgress.value,40);
  assert.match(e.recProgressHint.textContent,/상태 수신 끊김/);
});
