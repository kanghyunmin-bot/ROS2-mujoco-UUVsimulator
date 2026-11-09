"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const { test } = require("node:test");
const source = fs.readFileSync(path.join(__dirname, "../gui/web_static/real2sim-build.js"), "utf8");

test("build form validates input, posts requested paths and refreshes registry on completion", async () => {
  const nodes = {};
  const node = id => nodes[id] ||= {
    value: "", textContent: "", disabled: false, listeners: {},
    addEventListener(event, callback) { this.listeners[event] = callback; },
  };
  const requests = [];
  let job = { running: false, status: "ready", log: "", output: null, success: null };
  let refreshes = 0;
  let poll;
  const ctx = vm.createContext({
    window: { StationLanguage: { text: english => english } },
    document: { getElementById: node },
    setInterval: callback => { poll = callback; },
    refreshReal2SimVehicles: () => { refreshes++; },
    fetch: async (url, options) => {
      requests.push({ url, options });
      if (url === "/api/real2sim/build") job = { running: true, status: "running", log: "fitting", output: "example", success: null };
      return { ok: true, json: async () => url.endsWith("/status") ? job : { ok: true } };
    },
  });
  vm.runInContext(source, ctx);
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(node("real2simBuildStart").disabled, true);
  node("real2simPackage").value = "outputs/example-input";
  node("real2simOutputName").value = "../unsafe";
  node("real2simOutputName").listeners.input();
  assert.equal(node("real2simBuildStart").disabled, true);
  node("real2simOutputName").value = "example-fit";
  node("real2simOutputName").listeners.input();
  assert.equal(node("real2simBuildStart").disabled, false);
  await node("real2simBuildForm").listeners.submit({ preventDefault() {} });
  const request = requests.find(entry => entry.url === "/api/real2sim/build");
  assert.deepEqual(JSON.parse(request.options.body), { package: "outputs/example-input", output_name: "example-fit" });
  assert.equal(node("real2simBuildStop").disabled, false);
  assert.equal(node("real2simBuildLog").textContent, "fitting");
  job = { running: false, status: "complete", log: "finished", output: "example-fit", success: true };
  await poll();
  assert.equal(refreshes, 1);
  assert.equal(node("real2simBuildStop").disabled, true);
  assert.match(node("real2simBuildMessage").textContent, /completed/);
});
