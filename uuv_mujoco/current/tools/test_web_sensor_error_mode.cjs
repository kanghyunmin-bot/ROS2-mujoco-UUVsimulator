"use strict";
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const { test } = require("node:test");
const source = fs.readFileSync(path.join(__dirname, "../gui/web_static/app.js"), "utf8");
const render = source.slice(source.indexOf("function renderSimulationConfig("), source.indexOf("\nasync function startSimStack("));
const start = source.slice(source.indexOf("async function startSimStack("), source.indexOf("\nfunction renderStereoCamera("));

function setup() {
  const nodes = {};
  const node = id => nodes[id] ||= {
    value: "", checked: false, children: [], disabled: false,
    replaceChildren() { this.children = []; this.value = ""; },
    appendChild(child) { this.children.push(child); },
  };
  const state = { simPresetTouched: false, simPresetSignature: "", configurationLocked: false, simStopPending: false };
  const requests = [];
  const ctx = vm.createContext({
    state, $: node, window: { location: { port: "8878" } }, document: { activeElement: null, createElement: () => ({}) },
    setText: (id, text) => { node(id).textContent = text; },
    postCommand: async payload => { requests.push(payload); return {}; },
    pollStatus: async () => {},
  });
  vm.runInContext(render + "\n" + start, ctx);
  const config = {
    selected_preset_id: "research_pool_distributed",
    selected_sensor_error_mode: "existing",
    presets: ["research_pool_distributed", "course_current"].map(id => ({ id, label: id })),
    sensor_error_modes: [
      { id: "existing", description: "existing" },
      { id: "bag0402", description: "2 Hz; partial calibration" },
    ],
  };
  return { ctx, state, node, config, requests };
}

test("primary view shows pool only, advanced preserves selected comparison", () => {
  const { ctx, state, node, config } = setup();
  ctx.renderSimulationConfig(config, {});
  assert.equal(node("simLaunchPreset").children.length, 1);
  assert.equal(node("simLaunchPreset").value, "research_pool_distributed");
  node("showAdvancedEnvironments").checked = true;
  ctx.renderSimulationConfig(config, {});
  assert.equal(node("simLaunchPreset").children.length, 2);
  state.simPresetTouched = true;
  node("simLaunchPreset").value = "course_current";
  node("showAdvancedEnvironments").checked = false;
  ctx.renderSimulationConfig(config, {});
  assert.equal(node("simLaunchPreset").value, "course_current");
});

test("an external simulator links to the other GUI and disables local stop and reset", () => {
  const { ctx, node, config } = setup();
  ctx.renderSimulationConfig(config, { sim_running: true, sim_external: true });
  assert.equal(node("stackStopBtn").disabled, true);
  assert.equal(node("stackResetBtn").disabled, true);
  assert.equal(node("simOtherGuiLink").hidden, false);
  assert.equal(node("simOtherGuiLink").href, "http://127.0.0.1:18879/");
});

test("stop sends a shutdown command rather than a SITL reset", async () => {
  const { ctx, node, config, requests } = setup();
  ctx.renderSimulationConfig(config, { sim_running: true, sim_external: false });
  assert.equal(node("stackStopBtn").disabled, false);
  await ctx.stopSimStack();
  assert.equal(requests[0].command, "stack_stop");
});

test("VirtualGL selection survives idle polling and reaches the launch command", async () => {
  const { ctx, node, config, requests } = setup();
  node("virtualglViewer").checked = true;
  ctx.renderSimulationConfig(config, {});
  assert.equal(node("virtualglViewer").checked, true);
  await ctx.startSimStack();
  assert.equal(requests[0].virtualgl_viewer, true);
  ctx.renderSimulationConfig(config, { sim_running: true, virtualgl_viewer: true });
  assert.equal(node("virtualglViewer").checked, true);
  assert.equal(node("virtualglViewer").disabled, true);
  ctx.renderSimulationConfig(config, { virtualgl_available: false });
  assert.equal(node("virtualglViewer").checked, false);
  assert.equal(node("virtualglViewer").disabled, true);
});

test("error selection survives polling and locks while running", async () => {
  const { ctx, state, node, config, requests } = setup();
  ctx.renderSimulationConfig(config, {});
  state.sensorErrorTouched = true;
  node("sensorErrorMode").value = "bag0402";
  ctx.renderSimulationConfig(config, {});
  assert.match(node("sensorErrorModeStatus").textContent, /2 Hz/);
  await ctx.startSimStack();
  assert.equal(requests[0].sensor_error_mode, "bag0402");
  config.selected_sensor_error_mode = "bag0402";
  config.active_sensor_error = { mode: "bag0402" };
  ctx.renderSimulationConfig(config, { sim_running: true });
  assert.equal(node("sensorErrorMode").value, "bag0402");
  assert.equal(node("sensorErrorMode").disabled, true);
  assert.equal(node("showAdvancedEnvironments").disabled, true);
});
