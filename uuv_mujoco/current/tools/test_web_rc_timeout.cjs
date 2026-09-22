"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const { test } = require("node:test");
const source = fs.readFileSync(path.join(__dirname, "../gui/web_static/app.js"), "utf8");
const post = source.slice(source.indexOf("async function postRc("), source.indexOf("\nfunction setText("));
const flush = source.slice(source.indexOf("async function flushRcQueue("), source.indexOf("\nfunction queueRcRelease("));

test("a stalled RC request times out and the newest queued command can proceed", async () => {
  let calls = 0;
  let aborted = false;
  let cleared = 0;
  const state = { rcRequestInFlight: false, rcPending: { seq: 1 } };
  const context = vm.createContext({
    state, AbortController, document: { hidden: false },
    console: { error() {} },
    setTimeout: (fn) => setTimeout(fn, 10),
    clearTimeout: (id) => { cleared++; clearTimeout(id); },
    fetch: async (_url, options) => {
      calls++;
      if (calls === 1) {
        state.rcPending = { seq: 3 };
        return new Promise((_resolve, reject) => {
          options.signal?.addEventListener("abort", () => {
            aborted = true;
            reject(new Error("timeout"));
          });
        });
      }
      assert.equal(JSON.parse(options.body).seq, 3);
      return { ok: true, json: async () => ({ ok: true }) };
    },
  });
  vm.runInContext(post + "\n" + flush, context);
  const result = await Promise.race([
    context.flushRcQueue().then(() => "finished"),
    new Promise(resolve => setTimeout(() => resolve("stalled"), 100)),
  ]);
  assert.equal(result, "finished");
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(aborted, true);
  assert.equal(calls, 2);
  assert.equal(state.rcRequestInFlight, false);
  assert.equal(state.rcPending, null);
  assert.equal(cleared, 2);
});

test("pilot keepalive continues at neutral and stops when control is disabled", () => {
  const calls = [];
  const state = { rcEnabled: true, dragging: null, axes: { forward: 0, lateral: 0, heave: 0, yaw: 0 } };
  const heartbeat = source.slice(source.lastIndexOf("setInterval(() => {"));
  const context = vm.createContext({
    state, RC_KEEPALIVE_MS: 20, axesActive: () => false,
    sendRc: force => calls.push(force), setInterval: callback => callback(),
  });
  vm.runInContext(heartbeat, context);
  assert.deepEqual(calls, [true]);
  state.rcEnabled = false;
  vm.runInContext(heartbeat, context);
  assert.deepEqual(calls, [true]);
});

test("two controllers require selection and disconnect never switches to another pad", () => {
  let pads = [{index: 0, id: 'SHANWAN', connected: true}, {index: 1, id: 'Pro Controller', connected: true}];
  const state = {gamepadIndex: null};
  const context = vm.createContext({state, navigator: {getGamepads: () => pads}, $: () => null});
  const selection = source.slice(source.indexOf('function currentGamepad()'), source.indexOf('async function postCommand('));
  vm.runInContext(selection, context);
  assert.equal(context.currentGamepad(), null);
  state.gamepadIndex = 1;
  assert.equal(context.currentGamepad().id, 'Pro Controller');
  pads = [pads[0]];
  assert.equal(context.currentGamepad(), null);
  assert.equal(state.gamepadIndex, 1);
});
