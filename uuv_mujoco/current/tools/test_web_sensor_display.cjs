// Run with Node.js; no browser or ROS dependencies.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const script = fs.readFileSync(path.join(__dirname, '../gui/web_static/app.js'), 'utf8');
const start = script.indexOf('function fixed(');
const end = script.indexOf('function clamp(', start);
const context = vm.createContext({});
vm.runInContext(script.slice(start, end), context);
assert.equal(context.fixed(null), 'n/a');
assert.equal(context.fixed(0), '0.00');
assert.ok(Number.isNaN(context.telemetryNumber({depth: null}, 'depth')));
assert.ok(Number.isNaN(context.telemetryNumber({depth: ''}, 'depth')));
assert.equal(context.telemetryNumber({depth_m: null, depth: 2.5}, 'depth_m', 'depth'), 2.5);
assert.equal(context.telemetryNumber({depth: 0}, 'depth'), 0);
console.log('Sensor display: missing, fallback, and real zero passed.');

const sensorScript = fs.readFileSync(path.join(__dirname, '../gui/web_static/sensors.js'), 'utf8');
const sensorNodes = new Map();
const sensorContext = vm.createContext({window: {}, document: {getElementById: id => {
  if (!sensorNodes.has(id)) sensorNodes.set(id, {textContent: '', style: {}});
  return sensorNodes.get(id);
}}});
vm.runInContext(sensorScript, sensorContext);
sensorContext.window.renderStationSensors({
  telemetry: {sitl_mavlink_status: {sim_time_s: 12}, sitl_mavlink_status_age_s: .1},
  stereo_camera: {left: {age_s: .01, capture_stamp_s: 10}, right: {age_s: .01, capture_stamp_s: 11}},
});
assert.equal(sensorNodes.get('sensorCameras').textContent, '0.0 / 0.0 s');
assert.match(sensorNodes.get('sensorCameraCaptureAge').textContent, /2.00 \/ 1.00/);
assert.equal(sensorNodes.get('sensorCameraCaptureAge').style.color, '#ffbd69');
sensorContext.window.renderStationSensors({telemetry: {}, stereo_camera: {}});
assert.match(sensorNodes.get('sensorCameraCaptureAge').textContent, /— \/ —/);
assert.equal(sensorNodes.get('sensorCameraCaptureAge').style.color, '');
console.log('Camera receipt age and source capture delay remain distinct.');
