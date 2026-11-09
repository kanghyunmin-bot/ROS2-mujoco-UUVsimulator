// Browser preview pacing must never alter the ROS sensor publication contract.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const script = fs.readFileSync(path.join(__dirname, '../gui/web_static/app.js'), 'utf8');
const start = script.indexOf('function updateStereoView(');
const end = script.indexOf('function setStereoCameraExpanded(', start);
let now = 0;
const requests = [];
const image = {complete: true, removeAttribute() {}, set src(value) { requests.push(value); }};
const view = {classList: {add() {}, remove() {}}};
const state = {stereoSeq: {left: 0}, previewLastFrame: {left: -Infinity}};
const document = {hidden: false};
const previewRate = {hz: 4};
const context = vm.createContext({state, previewRate, previewVisible: () => !document.hidden, document, performance: {now: () => now},
  $: id => id.endsWith('Image') ? image : view});
vm.runInContext(script.slice(start, end), context);
const frame = seq => context.updateStereoView('left', {seq, available: true});
frame(1);
now = 100; frame(2);
assert.equal(requests.length, 1, '4fps preview must skip intervening 15Hz sensor frames');
now = 250; frame(3);
assert.equal(requests.length, 2);
assert.match(requests[1], /seq=3$/, 'preview must fetch newest frame instead of queuing old frames');
now = 500; image.complete = false; frame(4);
assert.equal(requests.length, 2, 'do not overlap an unfinished image download');
image.complete = true; document.hidden = true; now = 800; frame(5);
assert.equal(requests.length, 2, 'hidden document must not download images');
document.hidden = false; previewRate.hz = 0; frame(6);
assert.equal(requests.length, 2, 'pausing preview must not download images');
previewRate.hz = 10; frame(7);
assert.equal(requests.length, 3);
console.log('Camera preview: bounded pacing, latest frame, no overlap, pause and hidden-tab checks passed.');

// Fast frames can raise the preview cap, but sustained load should lower it
// more quickly. Low-rate and disconnected cameras cannot trigger an increase.
const rates = script.slice(script.indexOf('class PreviewRate {'), script.indexOf('const previewRate ='));
const rate = vm.runInNewContext(rates + '\nnew PreviewRate()');
let clock = 0;
function windowOf(left, right, slow = 0, lag = 0) {
  for (let i = 0; i < left; i++) rate.loaded('left', slow);
  for (let i = 0; i < right; i++) rate.loaded('right', slow);
  clock += 2000;
  rate.sample(clock, lag);
}
rate.sample(0, 0);
for (let i = 0; i < 3; i++) windowOf(8, 8);
assert.equal(rate.hz, 10);
windowOf(10, 6, 350);
assert.equal(rate.hz, 4);
windowOf(3, 3, 600);
assert.equal(rate.hz, 2);
for (let i = 0; i < 4; i++) windowOf(0, 0);
assert.equal(rate.hz, 2);
assert.deepEqual(Array.from(rate.fps), [0, 0]);
rate.setMode('4');
for (let i = 0; i < 4; i++) windowOf(8, 8, 600);
assert.equal(rate.hz, 4);
rate.setMode('0');
assert.equal(rate.hz, 0);
rate.setMode('auto');
assert.equal(rate.hz, 4);
rate.setMode('auto');
rate.sample(clock, 0);
for (let i = 0; i < 5; i++) windowOf(2, 2);
assert.equal(rate.hz, 4, 'a 1Hz source does not justify raising the cap');
windowOf(8, 8, 0, 500);
assert.equal(rate.hz, 2, 'main-thread stalls reduce the automatic cap');
rate.reset();
assert.equal(rate.start, null);
assert.deepEqual(Array.from(rate.fps), [0, 0]);
rate.setMode('invalid');
assert.equal(rate.mode, 'auto');
console.log('Adaptive preview: measured rate, recovery, load reduction and manual caps passed.');
