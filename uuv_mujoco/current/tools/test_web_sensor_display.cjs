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
