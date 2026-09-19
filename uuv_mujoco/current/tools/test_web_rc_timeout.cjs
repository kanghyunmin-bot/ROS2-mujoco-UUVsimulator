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
