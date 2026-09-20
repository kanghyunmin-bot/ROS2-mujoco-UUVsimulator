"use strict";
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../gui/web_static/app.js'), 'utf8');
const code = source.slice(source.indexOf('function clamp('), source.indexOf('function gamepadLabel('));
const ctx = vm.createContext({GAMEPAD_DEADZONE: 0.10});
vm.runInContext(code, ctx);
const low = [-0.913144322031312, -0.8170415356913968, -0.8948942533646657, -0.7967162083803827];
const high = [0.93465987121189, 0.9503769035920285, 0.836787011322367, 0.8607745597705008];
const pad = axes => ({id:'Nintendo Co., Ltd. Pro Controller (STANDARD GAMEPAD Vendor: 057e Product: 2009)', mapping:'standard',axes});
test('measured Pro Controller directional endpoints reach full commands', () => {
  assert.deepEqual({...ctx.physicalGamepadAxes(pad(low))}, {yaw:-1,heave:1,lateral:-1,forward:1});
  assert.deepEqual({...ctx.physicalGamepadAxes(pad(high))}, {yaw:1,heave:-1,lateral:1,forward:-1});
});
test('neutral deadzone stays neutral and larger inputs stay bounded', () => {
  for (const v of [0, .09, -.09]) {
    assert.ok(Object.values(ctx.physicalGamepadAxes(pad([v,v,v,v]))).every(x => x === 0));
  }
  assert.ok(Object.values(ctx.physicalGamepadAxes(pad([2,-2,2,-2]))).every(x => Math.abs(x) === 1));
});
test('another controller retains original scaling', () => {
  const result = ctx.physicalGamepadAxes({id:'SHANWAN Android Gamepad',axes:[.55,.55,.55,.55]});
  for (const v of Object.values(result)) assert.ok(Math.abs(Math.abs(v)-.5)<1e-12);
});
