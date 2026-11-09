const state = {
  rcEnabled: false,
  axes: { forward: 0, lateral: 0, heave: 0, yaw: 0 },
  lastRcSent: 0,
  rcSeq: 0,
  rcRequestInFlight: false,
  rcPending: null,
  pageReleaseSent: false,
  statusPollBusy: false,
  replayDuration: 0,
  dragging: null,
  joystickDrag: null,
  controlSource: "",
  gamepadEnabled: true,
  vlaControlPrepared: false,
  gamepadIndex: null,
  gamepadDriving: false,
  gamepadStatusText: "Gamepad: checking browser",
  toolKind: "",
  physicsRows: [],
  courseLayout: null,
  selectedCourseTarget: "",
  courseDragging: false,
  stereoCameraEnabled: true,
  stereoCameraVisionEnabled: true,
  stereoSeq: { left: 0, right: 0 },
  previewLastFrame: { left: -Infinity, right: -Infinity },
  cameraPollBusy: false,
  cameraPresetSignature: "",
  configurationLocked: false,
  simStartPending: false,
  simStartError: "",
  simStopPending: false,
  simVehicleSignature: "",
  simPresetSignature: "",
  simPresetChoice: "",
  real2simVehicles: [],
  real2simChoice: "",
  real2simSignature: "",
  real2simViewPending: false,
  real2simRunning: false,
  sensorErrorTouched: false,
  lastSimulationConfig: null,
  lastSimulationProcesses: null,
  pingerStartPending: false,
  pilotDock: null,
};

const COURSE_MARGIN = 34;
const COURSE_POINT_RADIUS = 7;
const STATUS_POLL_MS = 750;
const CAMERA_POLL_MS = 100;
const RC_SEND_MIN_INTERVAL_MS = 10;
const RC_KEEPALIVE_MS = 20;
const GAMEPAD_POLL_MS = 20;
const GAMEPAD_DEADZONE = 0.10;
const CLIENT_ID = `web-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 10)}`;

const $ = (id) => document.getElementById(id);
const tr = window.StationLanguage.text;

function fixed(value, digits = 2) {
  const number = value === null || value === undefined || value === "" ? NaN : Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "n/a";
}

function telemetryNumber(telemetry, ...keys) {
  for (const key of keys) {
    if (telemetry[key] === null || telemetry[key] === undefined || telemetry[key] === "") continue;
    const value = Number(telemetry[key]);
    if (Number.isFinite(value)) {
      return value;
    }
  }
  return Number.NaN;
}

function clamp(value, min = -1, max = 1) {
  return Math.max(min, Math.min(max, Number(value) || 0));
}

function applyGamepadDeadzone(value, limits = [-1, 1]) {
  const axis = clamp(value);
  const magnitude = Math.abs(axis);
  if (magnitude <= GAMEPAD_DEADZONE) {
    return 0;
  }
  // Preserve the raw neutral deadzone; independently scale each direction.
  const endpoint = axis < 0 ? -limits[0] : limits[1];
  return clamp(Math.sign(axis) * (magnitude - GAMEPAD_DEADZONE) / (endpoint - GAMEPAD_DEADZONE));
}

function gamepadAxisCalibration(gamepad) {
  // Local USB Pro Controller sweep, 2026-09-20. User confirmed full travel.
  // EV_ABS X/Y/RX/RY normalized by 32767; centre remains zero (not measured).
  const procon = /vendor:\s*057e.*product:\s*2009/i.test(gamepad.id || "")
    || /nintendo.*pro controller/i.test(gamepad.id || "");
  return procon ? [
    [-0.913144322031312, 0.93465987121189],
    [-0.8170415356913968, 0.9503769035920285],
    [-0.8948942533646657, 0.836787011322367],
    [-0.7967162083803827, 0.8607745597705008],
  ] : null;
}

function physicalGamepadAxes(gamepad) {
  const calibration = gamepadAxisCalibration(gamepad);
  const axis = (index) => applyGamepadDeadzone(gamepad.axes?.[index] ?? 0, calibration?.[index]);
  // Standard browser layout: left stick is axes 0/1, right stick is 2/3.
  // Browser Y grows downward, while GUI/ArduSub command Y grows upward.
  return {
    yaw: axis(0),
    heave: -axis(1),
    lateral: axis(2),
    forward: -axis(3),
  };
}

function gamepadLabel(gamepad) {
  const name = String(gamepad?.id || "controller").replace(/\s+/g, " ").trim();
  return name.length > 44 ? `${name.slice(0, 41)}...` : name;
}

function renderGamepadStatus(text) {
  state.gamepadStatusText = text;
  setText("gamepadStatus", text);
}

function currentGamepad() {
  if (!navigator.getGamepads) {
    return null;
  }
  const pads = Array.from(navigator.getGamepads()).filter(pad => pad && pad.connected !== false);
  // Multiple attached receivers must not silently choose the wrong controller.
  if (state.gamepadIndex === null && pads.length === 1) state.gamepadIndex = pads[0].index;
  const selector = $("gamepadDevice");
  const signature = JSON.stringify(pads.map(pad => [pad.index, pad.id]));
  if (selector && selector.dataset.devices !== signature) {
    selector.replaceChildren(new Option(tr("Select a controller", "조종기를 선택하세요"), ""));
    for (const pad of pads) selector.add(new Option(`${pad.index + 1}: ${pad.id}`, String(pad.index)));
    selector.dataset.devices = signature;
  }
  if (selector) selector.value = state.gamepadIndex === null ? "" : String(state.gamepadIndex);
  return pads.find(pad => pad.index === state.gamepadIndex) || null;
}

async function postCommand(payload) {
  const response = await fetch("/api/command", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const body = await response.json().catch(() => ({}));
  if (!response.ok || body.ok === false) {
    throw new Error(body.error || `HTTP ${response.status}`);
  }
  return body;
}

async function postRc(payload) {
  // Bound the single-flight queue even when the network or response body stalls.
  // The server's stale-input watchdog remains responsible for RC release.
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 500);
  try {
    const response = await fetch("/api/rc", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
      signal: controller.signal,
    });
    const body = await response.json().catch(() => ({}));
    if (!response.ok || body.ok === false) {
      throw new Error(body.error || `HTTP ${response.status}`);
    }
    return body;
  } finally {
    clearTimeout(timeout);
  }
}

function setText(id, value) {
  const el = $(id);
  if (el && el.textContent !== String(value ?? "")) {
    el.textContent = value ?? "";
  }
}

function setValue(id, value) {
  const el = $(id);
  if (el && document.activeElement !== el) {
    el.value = value ?? "";
  }
}

function renderStep(label, fn) {
  try {
    fn();
  } catch (error) {
    console.error(`render ${label} failed`, error);
  }
}

function axisValue(id) {
  return clamp($(id).value);
}

function readAxesFromSliders() {
  state.axes = {
    forward: axisValue("axisForward"),
    lateral: axisValue("axisLateral"),
    heave: axisValue("axisHeave"),
    yaw: axisValue("axisYaw"),
  };
  renderAxisValues();
  syncSticksFromAxes();
}

function writeAxesToSliders() {
  $("axisForward").value = String(state.axes.forward);
  $("axisLateral").value = String(state.axes.lateral);
  $("axisHeave").value = String(state.axes.heave);
  $("axisYaw").value = String(state.axes.yaw);
  renderAxisValues();
}

function renderAxisValues() {
  setText("forwardValue", fixed(state.axes.forward));
  setText("lateralValue", fixed(state.axes.lateral));
  setText("heaveValue", fixed(state.axes.heave));
  setText("yawValue", fixed(state.axes.yaw));
}

function setAxes(nextAxes, { send = true } = {}) {
  state.axes = {
    forward: clamp(nextAxes.forward ?? state.axes.forward),
    lateral: clamp(nextAxes.lateral ?? state.axes.lateral),
    heave: clamp(nextAxes.heave ?? state.axes.heave),
    yaw: clamp(nextAxes.yaw ?? state.axes.yaw),
  };
  writeAxesToSliders();
  syncSticksFromAxes();
  if (send) {
    sendRc(false);
  }
}

function setStickAxes(mapping, x, y, { send = true } = {}) {
  const next = { ...state.axes };
  next[mapping.x] = clamp(x);
  next[mapping.y] = clamp(y);
  state.axes = next;
  writeAxesToSliders();
  syncSticksFromAxes();
  if (send) {
    sendRc(true);
  }
}

function centerAxes({ send = true } = {}) {
  setAxes({ forward: 0, lateral: 0, heave: 0, yaw: 0 }, { send });
}

function axesActive() {
  return Object.values(state.axes).some((value) => Math.abs(Number(value) || 0) > 0.01);
}

function sendRc(force = false) {
  if (window.autoCollectionRunning) return;
  if (document.hidden) {
    return;
  }
  const now = performance.now();
  if (!force && now - state.lastRcSent < RC_SEND_MIN_INTERVAL_MS) {
    return;
  }
  state.lastRcSent = now;
  queueRcPayload({
    enabled: state.rcEnabled,
    axes: { ...state.axes },
    client_id: CLIENT_ID,
    seq: nextRcSequence(),
  });
}

function nextRcSequence() {
  state.rcSeq += 1;
  return state.rcSeq;
}

function queueRcPayload(payload) {
  // Keep at most one request on the wire.  Pointer events may arrive faster
  // than HTTP responses, so retain only the newest unsent command.
  state.rcPending = payload;
  flushRcQueue();
}

async function flushRcQueue() {
  if (state.rcRequestInFlight || !state.rcPending) {
    return;
  }
  const payload = state.rcPending;
  state.rcPending = null;
  state.rcRequestInFlight = true;
  try {
    await postRc(payload);
  } catch (error) {
    console.error(error);
  } finally {
    state.rcRequestInFlight = false;
    if (state.rcPending && !document.hidden) {
      flushRcQueue();
    }
  }
}

function queueRcRelease() {
  state.rcPending = null;
  queueRcPayload({
    enabled: false,
    release: true,
    axes: { forward: 0, lateral: 0, heave: 0, yaw: 0 },
    client_id: CLIENT_ID,
    seq: nextRcSequence(),
  });
}

function releaseRcForInactivePage() {
  if (state.pageReleaseSent) {
    return;
  }
  state.pageReleaseSent = true;
  state.rcEnabled = false;
  state.axes = { forward: 0, lateral: 0, heave: 0, yaw: 0 };
  state.rcPending = null;
  const payload = {
    enabled: false,
    release: true,
    axes: { ...state.axes },
    client_id: CLIENT_ID,
    seq: nextRcSequence(),
  };
  const body = JSON.stringify(payload);
  let beaconSent = false;
  try {
    beaconSent = navigator.sendBeacon(
      "/api/rc",
      new Blob([body], { type: "application/json" })
    );
  } catch (_error) {
    beaconSent = false;
  }
  if (!beaconSent) {
    fetch("/api/rc", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body,
      keepalive: true,
    }).catch(() => {});
  }
}

function enablePilotInputFromJoystick() {
  if (window.demoResetPending) return;
  if (!state.rcEnabled) {
    state.rcEnabled = true;
    $("rcEnabled").checked = true;
  }
}

function releaseInput() {
  state.rcEnabled = false;
  $("rcEnabled").checked = false;
  state.controlSource = "";
  state.gamepadDriving = false;
  finishJoystickDrag(state.joystickDrag, null, { center: true, force: true });
  centerAxes({ send: false });
  queueRcRelease();
}

function releasePhysicalGamepad(reason) {
  if (state.controlSource !== "gamepad" && !state.gamepadDriving) {
    return;
  }
  state.gamepadDriving = false;
  state.controlSource = "";
  // A disconnected controller must never leave a non-neutral override active.
  // Reuse the virtual-stick release path so ownership and MAVROS release
  // semantics remain identical for both pilot inputs.
  releaseInput();
  renderGamepadStatus(`Gamepad: ${reason}`);
}

function pollPhysicalGamepad() {
  if (state.vlaControlPrepared) return;
  if (document.hidden) {
    return;
  }
  const enabledToggle = $("gamepadEnabled");
  state.gamepadEnabled = Boolean(enabledToggle?.checked);
  if (!state.gamepadEnabled) {
    if (state.controlSource === "gamepad" || state.gamepadDriving) {
      releasePhysicalGamepad("disabled");
    }
    renderGamepadStatus("Gamepad: disabled");
    return;
  }
  if (!navigator.getGamepads) {
    renderGamepadStatus("Gamepad: browser unsupported");
    return;
  }
  const gamepad = currentGamepad();
  if (!gamepad) {
    if (state.controlSource === "gamepad" || state.gamepadDriving) {
      releasePhysicalGamepad("disconnected");
    } else {
      renderGamepadStatus(tr("Gamepad: select a controller and move a stick or press a button", "Gamepad: 조종기를 선택하고 버튼 또는 스틱을 움직이세요"));
    }
    return;
  }

  const axes = physicalGamepadAxes(gamepad);
  const active = Object.values(axes).some((value) => Math.abs(value) > 0.001);
  renderGamepadStatus(
    active
      ? `Gamepad: ${gamepadLabel(gamepad)} — RC active${gamepadAxisCalibration(gamepad) ? tr(" · Pro Controller range correction", " · 프로콘 범위 보정") : ""}`
      : `Gamepad: ${gamepadLabel(gamepad)} — centered${gamepadAxisCalibration(gamepad) ? tr(" · Pro Controller range correction", " · 프로콘 범위 보정") : ""}`
  );
  // Leave a virtual-stick pilot session untouched while a connected physical
  // gamepad remains centred. Once the operator moves a physical stick, it
  // becomes the sole active pilot source until it returns to neutral.
  if (!active && state.controlSource !== "gamepad" && !state.gamepadDriving) {
    return;
  }

  const wasDriving = state.gamepadDriving;
  state.controlSource = "gamepad";
  if (active) {
    enablePilotInputFromJoystick();
  }
  setAxes(axes, { send: false });
  // This also publishes one neutral frame after stick release.
  sendRc(true);
  state.gamepadDriving = active;
  if (!active && wasDriving) {
    state.controlSource = "";
  }
}

function joystickVectorFromEvent(pad, event) {
  const rect = pad.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const radius = Math.max(1, Math.min(rect.width, rect.height) * 0.42);
  return {
    x: clamp((event.clientX - cx) / radius),
    y: clamp((cy - event.clientY) / radius),
  };
}

function joystickPointerMatches(session, event) {
  return !event || event.pointerId === session.pointerId;
}

function applyJoystickDrag(session, event, { force = false } = {}) {
  if (!session || !joystickPointerMatches(session, event)) {
    return;
  }
  if (event.cancelable) {
    event.preventDefault();
  }
  const vector = joystickVectorFromEvent(session.pad, event);
  setStickAxes(session.mapping, vector.x, vector.y, { send: false });
  sendRc(force);
}

function finishJoystickDrag(session = state.joystickDrag, event = null, { center = true, force = true } = {}) {
  if (!session || state.joystickDrag !== session) {
    return;
  }
  if (event && !joystickPointerMatches(session, event)) {
    return;
  }
  if (event?.cancelable) {
    event.preventDefault();
  }
  window.removeEventListener("pointermove", session.onMove, true);
  window.removeEventListener("pointerup", session.onFinish, true);
  window.removeEventListener("pointercancel", session.onFinish, true);
  window.removeEventListener("blur", session.onBlur);
  session.pad.classList.remove("dragging");
  try {
    session.pad.releasePointerCapture(session.pointerId);
  } catch (_error) {
    // Pointer may already be released by the browser.
  }
  state.joystickDrag = null;
  state.dragging = null;
  if (center) {
    setStickAxes(session.mapping, 0, 0, { send: false });
    sendRc(force);
    if (state.controlSource === "virtual" && !axesActive()) {
      state.controlSource = "";
    }
  }
}

function startJoystickDrag(pad, mapping, event) {
  if (state.vlaControlPrepared) return;
  if (event.button !== undefined && event.button !== 0) {
    return;
  }
  if (event.cancelable) {
    event.preventDefault();
  }
  event.stopPropagation();
  finishJoystickDrag(state.joystickDrag, null, { center: true, force: true });
  state.controlSource = "virtual";
  state.gamepadDriving = false;
  const session = {
    pad,
    mapping,
    pointerId: event.pointerId,
    onMove: null,
    onFinish: null,
    onBlur: null,
  };
  session.onMove = (moveEvent) => applyJoystickDrag(session, moveEvent);
  session.onFinish = (finishEvent) => finishJoystickDrag(session, finishEvent, { center: true, force: true });
  session.onBlur = () => finishJoystickDrag(session, null, { center: true, force: true });
  state.joystickDrag = session;
  state.dragging = pad.id;
  pad.classList.add("dragging");
  try {
    pad.setPointerCapture(event.pointerId);
  } catch (_error) {
    // Window-level listeners below still keep dragging alive.
  }
  window.addEventListener("pointermove", session.onMove, { capture: true, passive: false });
  window.addEventListener("pointerup", session.onFinish, { capture: true, passive: false });
  window.addEventListener("pointercancel", session.onFinish, { capture: true, passive: false });
  window.addEventListener("blur", session.onBlur);
  enablePilotInputFromJoystick();
  applyJoystickDrag(session, event, { force: true });
}

function bindJoystick(pad, mapping) {
  pad.addEventListener("pointerdown", (event) => startJoystickDrag(pad, mapping, event), { passive: false });
  pad.addEventListener("dragstart", (event) => event.preventDefault());
  pad.addEventListener("contextmenu", (event) => event.preventDefault());
}

function syncSticksFromAxes() {
  positionKnob($("leftStick").querySelector(".stick-knob"), state.axes.yaw, state.axes.heave);
  positionKnob($("rightStick").querySelector(".stick-knob"), state.axes.lateral, state.axes.forward);
}

function positionKnob(knob, x, y) {
  const pad = knob.parentElement;
  const travel = Math.max(36, Math.min(pad.clientWidth, pad.clientHeight) * 0.34);
  knob.style.transform = `translate(calc(-50% + ${x * travel}px), calc(-50% + ${-y * travel}px))`;
}

function setToggle(targetId, buttonId, openText, closeText) {
  const target = $(targetId);
  const button = $(buttonId);
  target.classList.toggle("hidden");
  button.textContent = target.classList.contains("hidden") ? openText : closeText;
  button.setAttribute("aria-expanded", String(!target.classList.contains("hidden")));
  button.setAttribute("aria-controls", targetId);
}

function renderStatus(payload) {
  state.configurationLocked = Boolean(payload.recorder?.configuration_locked);
  if (window.renderRecorder) window.renderRecorder(payload.recorder || {});
  if (window.renderStationSensors) window.renderStationSensors(payload);
  const telemetry = payload.telemetry || {};
  const ui = payload.ui || {};
  const processes = payload.processes || {};
  const control = payload.control || {};
  state.vlaControlPrepared = Boolean(control.vla_prepared);
  $("vlaPrepareControlBtn").disabled = state.vlaControlPrepared || state.configurationLocked;
  $("vlaRestoreManualBtn").disabled = !state.vlaControlPrepared || state.configurationLocked;
  if (state.vlaControlPrepared) {
    state.rcEnabled = false;
    state.rcPending = null;
    setText("vlaControlStatus", tr("VLA control ready · restore manual control after stopping the policy", "VLA 제어권 준비됨 · 정책 종료 후 수동 복귀"));
  }

  const tools = payload.tools || {};

  if (typeof control.enabled === "boolean") {
    // Status is observation, not permission to transmit. Only an explicit
    // local input may enable this tab; otherwise an idle viewer can keep
    // sending neutral frames after another controller's watchdog expires.
    const otherOwner = Boolean(control.owner && control.owner !== CLIENT_ID);
    state.rcEnabled = state.rcEnabled && control.enabled && !otherOwner && !state.vlaControlPrepared;
    if (!state.rcEnabled) state.rcPending = null;
    const rcToggle = $("rcEnabled");
    if (rcToggle && document.activeElement !== rcToggle) {
      rcToggle.checked = state.rcEnabled;
    }
  }

  setText("simStackPill", ui.sim_stack_status || "sim: stopped");
  setText("rosPkgPill", ui.ros_pkg_status || "mavros: stopped");
  setText("ping360Pill", ui.ping360_summary || "ping360: no status");
  setText("vehicleSummary", ui.vehicle_summary || "vehicle: disconnected");
  setText("motionSummary", Number.isFinite(telemetryNumber(telemetry, "imu_age_s")) ? (ui.motion_summary || "motion: n/a") : tr("Waiting for IMU · attitude unavailable", "IMU 미수신 · 자세 정보 없음"));
  setText("controlSummaryLeft", ui.control_summary || "control: idle");
  setText("statusText", ui.status || "disconnected");
  setText("modeText", ui.mode || "mode: UNKNOWN");
  setText("batteryText", ui.battery || "battery: n/a");
  setText("poseText", ui.pose || "pose: n/a");
  setText("velocityText", ui.velocity || "velocity: n/a");
  setText("imuText", ui.imu || "imu: n/a");
  setText("autopilotText", ui.autopilot || "autopilot: n/a");
  setText("depthTargetText", ui.depth_target || "depth: n/a");
  setText("depthSourceText", ui.depth_source || "depth source: unavailable");
  setText("ageText", ui.age || "state age: n/a");
  setText("controlSummary", ui.control_summary || "control: idle");
  setText("controlText", ui.control || "setpoint: x=0.00 y=0.00 z=0.00 yaw=0.00");
  setText("rcOverrideText", ui.rc_override || "pilot input: off");
  setText("commandReady", ui.command_ready || "WAIT: vehicle");
  setText("simStackStatus", state.simStartError || ui.sim_stack_status || "sim: stopped");
  setText("rosPkgStatus", ui.ros_pkg_status || "mavros: stopped");
  setText("rvizStatus", ui.rviz_status || "rviz: stopped");
  setText("pingerHomingStatus", ui.pinger_homing_status || processes.pinger_homing_status || "pinger homing: stopped");
  setText("ping360Summary", ui.ping360_summary || "ping360: no status");
  setText("ping360ViewStatus", ui.ping360_view_status || "ping360 view: closed");
  setText("rcReplayStatus", tools.rc_replay_status || "replay: unloaded");
  setText("rcReplayTime", tools.rc_replay_time || "00:00.0 / 00:00.0");
  setText("physicsStatus", tools.physics_status || "physics params [current]: idle");
  setText("courseStatus", tools.buoy_layout_status || "course layout: idle");
  setText("physicsPath", `path: ${tools.physics_profile_path || "loading"}`);
  setText("coursePath", `path: ${tools.course_scene_path || "loading"}`);
  renderStep("attitude", () => drawAttitude(telemetry));
  renderStep("depth", () => drawDepth(telemetry));
  renderStep("camera config", () => renderCameraConfig(payload.camera_config || processes.camera_config || {}));
  renderStep("simulation config", () => renderSimulationConfig(processes.simulation_config || {}, processes));
  renderStep("BlueROV2 motor output", () => renderBlueRovMotors(telemetry));
  renderStep("stereo camera", () => renderStereoCamera(payload.stereo_camera || {}));

  if (document.body?.dataset?.vehicle !== "bluerov2_standard") {
    setValue("rcReplayPath", tools.rc_replay_path || "");
  }
  setValue("rcReplayRate", tools.rc_replay_rate || "1.0");
  setValue("fcuUrlInput", processes.ros_pkg_fcu_url || "udp://0.0.0.0:14551@");
  setText("rosPkgSource", `rospkg read-only source: ${payload.server?.ros_package_dir || ""}`);

  $("mavrosToggleBtn").textContent = processes.mavros_running ? "MAVROS OFF" : "MAVROS ON";
  $("rvizToggleBtn").textContent = processes.rviz_running ? "RViz OFF" : "RViz ON";
  const pingerStartButton = $("pingerHomingStartBtn");
  const pingerRunning = Boolean(processes.pinger_homing_running);
  pingerStartButton.textContent = pingerRunning
    ? "Pinger running"
    : state.pingerStartPending
      ? "Pinger starting"
      : "Start pinger";
  pingerStartButton.disabled = state.configurationLocked || pingerRunning || state.pingerStartPending;

  if (Number.isFinite(Number(tools.rc_replay_duration_s))) {
    state.replayDuration = Number(tools.rc_replay_duration_s);
    $("rcReplaySlider").max = String(Math.max(state.replayDuration, 1));
  }
  if (document.activeElement !== $("rcReplaySlider") && Number.isFinite(Number(tools.rc_replay_position_s))) {
    $("rcReplaySlider").value = String(Number(tools.rc_replay_position_s));
  }

  renderStep("rc feedback", () => renderRcFeedback(telemetry));
  renderStep("events", () => renderEvents(telemetry.events || []));
}

function renderSimulationConfig(config, processes) {
  state.lastSimulationConfig = config;
  state.lastSimulationProcesses = processes;
  const allPresets = config.presets || [];
  const vehicleSelect = $("simVehicle");
  const select = $("simLaunchPreset");
  const advanced = $("showAdvancedEnvironments");
  const running = Boolean(processes?.sim_running);
  const external = Boolean(processes?.sim_external);
  const locked = state.configurationLocked || state.simStartPending || state.simStopPending || running || external;
  const active = running && !external ? config.active_preset_id : "";
  const selected = allPresets.find(preset => preset.id === (active || state.simPresetChoice))
    || allPresets.find(preset => preset.id === config.selected_preset_id)
    || allPresets[0];

  const vehicles = [...new Set(allPresets.map(preset => preset.vehicle))];
  const importedVehicles = state.real2simVehicles || [];
  const selectedReal2Sim = importedVehicles.find(vehicle => vehicle.id === state.real2simChoice);
  const signature = JSON.stringify([vehicles, importedVehicles.map(vehicle => [vehicle.id, vehicle.name, vehicle.output])]);
  if (signature && signature !== state.simVehicleSignature) {
    vehicleSelect.replaceChildren(
      ...vehicles.map(vehicle => new Option(vehicleLabel(vehicle), vehicle)),
      ...importedVehicles.map(vehicle => {
        const duplicate = importedVehicles.some(other => other.id !== vehicle.id && other.name === vehicle.name);
        const label = duplicate && vehicle.output ? vehicle.output : vehicle.name;
        return new Option(`${label} · Real2Sim`, `real2sim:${vehicle.id}`);
      })
    );
    state.simVehicleSignature = signature;
  }
  if (selectedReal2Sim) vehicleSelect.value = `real2sim:${selectedReal2Sim.id}`;
  else if (selected) vehicleSelect.value = selected.vehicle;
  const vehiclePresets = allPresets.filter(preset => preset.vehicle === selected?.vehicle);
  const presets = vehiclePresets.filter(preset => advanced.checked
    || preset === selected || preset === vehiclePresets[0]);
  const presetSignature = presets.map(preset => preset.id).join("|");
  if (presetSignature && presetSignature !== state.simPresetSignature) {
    select.replaceChildren(...presets.map(preset => {
      const option = new Option(tr(preset.label_en || preset.label, preset.label), preset.id);
      option.title = tr(preset.description_en || preset.description, preset.description);
      return option;
    }));
    state.simPresetSignature = presetSignature;
  }
  if (selected) {
    select.value = selected.id;
    const vehicleName = selectedReal2Sim ? selectedReal2Sim.name : vehicleLabel(selected.vehicle);
    window.setWorkspaceVehicle?.(selectedReal2Sim ? "real2sim" : selected.vehicle, selectedReal2Sim?.name);
    setText("workspaceVehicle", `${vehicleName} · ${tr("Underwater Robotics Workspace", "수중 로봇 실험 워크스페이스")}`);
    document.title = `${vehicleName} · Research Studio`;
    const label = select.selectedOptions[0]?.textContent || selected.id;
    const activeText = active ? tr(" · active", " · 실행 중") : "";
    setText("simLaunchPresetStatus", `${label} · ${selected.profile}/${selected.fluid_model}${activeText}`);
  }
  vehicleSelect.disabled = locked || !selected || vehicles.length + importedVehicles.length < 2;
  select.disabled = locked || !selected || Boolean(selectedReal2Sim);
  advanced.disabled = locked || vehiclePresets.length < 2 || Boolean(selectedReal2Sim);
  renderReal2SimPanel(selectedReal2Sim, locked);

  const errorSelect = $("sensorErrorMode");
  if (!state.sensorErrorTouched) errorSelect.value = config.selected_sensor_error_mode || "existing";
  const blue = selected?.vehicle === "bluerov2_standard";
  const aprilBagOption = errorSelect.querySelector('[value="bag0402"]');
  aprilBagOption.disabled = blue;
  if (blue && errorSelect.value === "bag0402") errorSelect.value = "existing";
  errorSelect.disabled = locked || !selected || Boolean(selectedReal2Sim);
  const errorMode = (config.sensor_error_modes || []).find(mode => mode.id === errorSelect.value);
  const activeError = config.active_sensor_error?.mode;
  const errorDescription = errorMode
    ? tr(errorMode.description_en || errorMode.description, errorMode.description)
    : tr("Use existing sensor settings", "기존 센서 설정 유지");
  setText("sensorErrorModeStatus", errorDescription + (activeError
    ? tr(` · active: ${activeError}`, ` · 실행 중: ${activeError}`)
    : tr(" · applies on next start", " · 다음 시작에 적용")));

  $("stackStartBtn").disabled = locked || !selected || Boolean(selectedReal2Sim);
  $("stackStartBtn").textContent = external ? tr("External Sim Running", "외부 시뮬 실행 중")
    : running ? tr("Simulation Running", "시뮬 실행 중")
    : state.simStartPending ? tr("Starting…", "시작 중…") : tr("Start Simulation", "시뮬 시작");
  $("stackStopBtn").disabled = state.configurationLocked || !running || external || state.simStartPending || state.simStopPending;
  $("stackStopBtn").textContent = state.simStopPending ? tr("Stopping…", "정지 중…") : tr("Stop Simulation", "시뮬 정지");
  $("stackResetBtn").disabled = state.configurationLocked || external || state.simStartPending || state.simStopPending || Boolean(selectedReal2Sim);
  const otherGuiLink = $("simOtherGuiLink");
  const otherGui = {
    "8878": ["18879", tr("Open Research GUI ↗", "연구 GUI 열기 ↗")],
    "18879": ["8878", tr("Open Main GUI ↗", "기본 GUI 열기 ↗")],
  }[window.location.port];
  otherGuiLink.hidden = !external || !otherGui;
  if (external && otherGui) {
    const [port, label] = otherGui;
    otherGuiLink.href = `http://127.0.0.1:${port}/`;
    otherGuiLink.textContent = label;
  }
  const virtualglOption = $("virtualglViewer");
  const virtualglUnavailable = processes?.virtualgl_available === false;
  virtualglOption.disabled = locked || !selected || virtualglUnavailable || Boolean(selectedReal2Sim);
  if (running || virtualglUnavailable) virtualglOption.checked = Boolean(processes?.virtualgl_viewer);
  const viewerHelp = {
    display_or_gpu: tr("VirtualGL requires a Linux X11 desktop and access to the NVIDIA GPU.",
      "VirtualGL을 사용하려면 Linux X11 화면과 NVIDIA GPU 연결이 필요합니다."),
    missing_install: tr("Install VirtualGL before using the GPU viewer.",
      "GPU 뷰어를 사용하려면 VirtualGL을 먼저 설치하세요."),
  }[processes?.virtualgl_unavailable_reason];
  $("virtualglOption").title = viewerHelp || (running || external
    ? tr("Stop the simulation before changing the GPU viewer option.", "GPU 뷰어 설정을 바꾸려면 시뮬레이션을 먼저 정지하세요.")
    : tr("Open the MuJoCo desktop viewer using VirtualGL.", "VirtualGL로 MuJoCo 데스크톱 뷰어를 엽니다."));
}

function renderReal2SimPanel(vehicle, locked) {
  const panel = $("real2simPanel");
  if (!panel) return;
  panel.hidden = !vehicle;
  if (!vehicle) return;
  const sensorNames = { imu: "IMU", pressure: tr("Pressure", "압력"), dvl: "DVL",
    camera_left: tr("Left camera", "왼쪽 카메라"), camera_right: tr("Right camera", "오른쪽 카메라") };
  const sensors = (vehicle.sensors || []).map(name => sensorNames[name] || name);
  setText("real2simSensors", `${tr("Configured sensors", "설정된 센서")} · ${sensors.join(", ") || tr("None", "없음")}`);
  $("real2simReview").hidden = !vehicle.needs_review;
  const trialSelect = $("real2simTrial");
  const profileSelect = $("real2simProfile");
  const signature = JSON.stringify([vehicle.id, vehicle.trials, vehicle.profiles]);
  if (state.real2simSignature !== signature) {
    trialSelect.replaceChildren(...(vehicle.trials || []).map(name => new Option(name, name)));
    profileSelect.replaceChildren(...(vehicle.profiles || []).map(name => new Option(name, name)));
    trialSelect.value = vehicle.trials?.[0] || "";
    profileSelect.value = vehicle.profiles?.includes("candidate") ? "candidate" : (vehicle.profiles?.[0] || "");
    state.real2simSignature = signature;
  }
  trialSelect.disabled = locked || state.real2simViewPending;
  profileSelect.disabled = locked || state.real2simViewPending;
  $("real2simViewBtn").disabled = locked || state.real2simViewPending || state.real2simRunning || !trialSelect.value || !profileSelect.value;
  $("real2simStopBtn").disabled = state.real2simViewPending || !state.real2simRunning;
}

async function refreshReal2SimVehicles() {
  try {
    const response = await fetch("/api/real2sim/vehicles", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const data = await response.json();
    state.real2simVehicles = Array.isArray(data.vehicles) ? data.vehicles : [];
    const statusResponse = await fetch("/api/real2sim/status", { cache: "no-store" });
    if (statusResponse.ok && !state.real2simViewPending) {
      const status = await statusResponse.json();
      if (state.real2simRunning && !status.running) {
        setText("real2simStatus", status.status === "Viewer finished"
          ? tr("Replay finished.", "재생이 끝났습니다.")
          : status.status);
      }
      state.real2simRunning = Boolean(status.running);
    }
    if (state.real2simChoice && !state.real2simVehicles.some(vehicle => vehicle.id === state.real2simChoice)) {
      state.real2simChoice = "";
    }
    if (state.lastSimulationConfig) renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  } catch (error) {
    setText("real2simStatus", tr(`Vehicle list unavailable: ${error.message}`, `기체 목록을 불러오지 못했습니다: ${error.message}`));
  }
}

async function openReal2SimReplay() {
  if (!state.real2simChoice || $("real2simViewBtn").disabled) return;
  state.real2simViewPending = true;
  renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  try {
    const response = await fetch("/api/real2sim/view", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ id: state.real2simChoice, trial: $("real2simTrial").value,
        profile: $("real2simProfile").value, playback_rate: 1.0 }),
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok || result.ok === false) throw new Error(result.error || `HTTP ${response.status}`);
    state.real2simRunning = true;
    setText("real2simStatus", tr("MuJoCo replay opened.", "MuJoCo 재생 창을 열었습니다."));
  } catch (error) {
    setText("real2simStatus", tr(`Replay failed: ${error.message}`, `재생 실패: ${error.message}`));
  } finally {
    state.real2simViewPending = false;
    renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  }
}

async function stopReal2SimReplay() {
  if (!state.real2simRunning || state.real2simViewPending) return;
  state.real2simViewPending = true;
  renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  try {
    const response = await fetch("/api/real2sim/stop", { method: "POST" });
    const result = await response.json().catch(() => ({}));
    if (!response.ok || result.ok === false) throw new Error(result.error || `HTTP ${response.status}`);
    state.real2simRunning = false;
    setText("real2simStatus", tr("Replay stopped.", "재생을 종료했습니다."));
  } catch (error) {
    setText("real2simStatus", tr(`Stop failed: ${error.message}`, `종료 실패: ${error.message}`));
  } finally {
    state.real2simViewPending = false;
    renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  }
}

function vehicleLabel(vehicle) {
  return { bluerov2_standard: "BlueROV2 Standard", ut27: "UT27" }[vehicle] || vehicle;
}

async function startSimStack() {
  if ($("stackStartBtn").disabled) return;
  const payload = {
    command: "stack_start",
    sim_preset: $("simLaunchPreset").value,
    sensor_error_mode: $("sensorErrorMode").value,
    virtualgl_viewer: $("virtualglViewer").checked,
  };
  state.simStartPending = true;
  state.simStartError = "";
  renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  setText("simStackStatus", tr("Simulation start requested", "시뮬레이션 시작 요청"));
  try {
    const result = await postCommand(payload);
    if (!result.simulation_config) throw new Error(result.status || tr("Start rejected", "시작 요청 거부"));
    state.simPresetChoice = "";
    state.sensorErrorTouched = false;
  } catch (error) {
    state.simStartError = tr(`Start failed: ${error.message}`, `시작 실패: ${error.message}`);
    setText("simStackStatus", state.simStartError);
  } finally {
    state.simStartPending = false;
    await pollStatus();
  }
}

async function stopSimStack() {
  if (state.simStopPending) return;
  state.simStopPending = true;
  renderSimulationConfig(state.lastSimulationConfig || {}, state.lastSimulationProcesses);
  try {
    await postCommand({ command: "stack_stop" });
  } finally {
    state.simStopPending = false;
    await pollStatus();
  }
}

function renderStereoCamera(camera) {
  const enabled = camera.enabled !== false;
  state.stereoCameraEnabled = enabled;
  const toggle = $("stereoCameraEnabled");
  if (toggle && document.activeElement !== toggle) {
    toggle.checked = enabled;
  }
  const frame = camera.left || {};
  const visionEnabled = camera.display_mode === "vision";
  state.stereoCameraVisionEnabled = visionEnabled;
  const visionToggle = $("stereoCameraVisionEnabled");
  if (visionToggle && document.activeElement !== visionToggle) {
    visionToggle.checked = visionEnabled;
  }
  if (!enabled) {
    updateStereoView("left", { available: false, seq: 0 });
    updateStereoView("right", { available: false, seq: 0 });
    setText("stereoCameraStatus", "camera: off");
    return;
  }
  updateStereoView("left", frame);
  updateStereoView("right", camera.right || {});
  const errorText = camera.error ? ` | ${camera.error}` : "";
  const source = camera.display_source === "vision" ? "vision" : "raw";
  const fallback = camera.vision_fallback ? " (waiting for overlay)" : "";
  const detectionStatus = document.body?.dataset?.vehicle === "bluerov2_standard" ? "" : yoloStatusText(camera.detection);
  setText(
    "stereoCameraStatus",
    `camera: ${stereoStatusText(frame)} | view: ${source}${fallback}${detectionStatus}${errorText}`
  );
}

function yoloStatusText(detection) {
  if (!detection) {
    return "";
  }
  if (!detection.enabled) {
    return " | yolo: off";
  }
  if (!detection.model_found) {
    return " | yolo: model missing";
  }
  if (detection.error) {
    return ` | yolo: ${detection.error}`;
  }
  const count = Number(detection.count) || 0;
  const candidateCount = Number(detection.candidate_count) || 0;
  const timeMs = Number(detection.last_inference_ms);
  const timeText = Number.isFinite(timeMs) && timeMs > 0 ? `, ${fixed(timeMs, 0)} ms` : "";
  const first = Array.isArray(detection.detections) ? detection.detections[0] : null;
  const center = Array.isArray(first?.center_px) ? first.center_px : null;
  const centerText = center ? ` @ ${fixed(center[0], 0)},${fixed(center[1], 0)}` : "";
  const vision = detection.vision || {};
  const command = vision.command || {};
  const visionState = vision.state ? ` | vision: ${vision.state}` : "";
  const commandText = vision.state
    ? ` f=${fixed(Number(command.forward) || 0, 2)} h=${fixed(Number(command.heave) || 0, 2)} yaw=${fixed(Number(command.yaw) || 0, 2)}`
    : "";
  const targetText = first ? ` | target: ${first.label || "buoy"}${centerText}` : " | target: none";
  return ` | yolo: ${count} raw / ${candidateCount} candidates${timeText}${targetText}${visionState}${commandText}`;
}

function renderCameraConfig(config) {
  const select = $("stereoCameraProfile");
  if (!select) {
    return;
  }
  const presets = Array.isArray(config.presets) ? config.presets : [];
  const signature = (document.body?.dataset?.vehicle || "ut27") + "|" + presets
    .map((preset) => `${preset.id}:${preset.width}x${preset.height}@${preset.hz}`)
    .join("|");
  if (signature && signature !== state.cameraPresetSignature) {
    select.innerHTML = "";
    for (const preset of presets) {
      const option = document.createElement("option");
      option.value = String(preset.id || "");
      option.textContent = cameraConfigLabel(preset);
      select.appendChild(option);
    }
    state.cameraPresetSignature = signature;
  }
  const presetId = String(config.preset_id || "");
  if (presetId && !Array.from(select.options).some((option) => option.value === presetId)) {
    const option = document.createElement("option");
    option.value = presetId;
    option.textContent = cameraConfigLabel(config);
    select.appendChild(option);
  }
  if (presetId && document.activeElement !== select) {
    select.value = presetId;
  }
  const optics = $("stereoCameraOptics");
  if (optics && document.activeElement !== optics) optics.value = config.optics_profile || "inherited";
  const launched = config.launched ? tr(` · requested at launch: ${cameraConfigLabel(config.launched)}`, ` · 실행 요청: ${cameraConfigLabel(config.launched)}`) : tr(" · applied profile unknown", " · 실행값 미확인");
  setText("stereoCameraConfigStatus", tr(`Selected sensor profile: ${cameraConfigLabel(config)}${launched} · see the preview status for measured rates`, `센서 선택: ${cameraConfigLabel(config)}${launched} · 실측 주기는 별도 확인`));
}

function cameraConfigLabel(config) {
  if (config && config.label) {
    const label = String(config.label);
    return document.body?.dataset?.vehicle === "bluerov2_standard" ? label.replace(/^VLA lite · /, "") : label;
  }
  const width = Number(config?.width) || 0;
  const height = Number(config?.height) || 0;
  const hz = Number(config?.hz) || 0;
  if (width > 0 && height > 0 && hz > 0) {
    const hzText = Number.isInteger(hz) ? String(hz) : hz.toFixed(1);
    return `${width}x${height} @ ${hzText}Hz`;
  }
  return "n/a";
}

function selectedCameraPresetPayload() {
  return {
    preset_id: $("stereoCameraProfile").value,
    optics_profile: $("stereoCameraOptics").value,
  };
}

function selectedCameraPresetLabel() {
  const select = $("stereoCameraProfile");
  return select.options[select.selectedIndex]?.textContent || "n/a";
}

async function applyCameraConfig(restart) {
  setText("stereoCameraConfigStatus", restart ? "profile: restarting" : "profile: saving");
  const body = await postCommand({
    command: "camera_config",
    values: selectedCameraPresetPayload(),
    restart,
  });
  const config = body.camera_config || {};
  setText("stereoCameraConfigStatus", `profile: ${cameraConfigLabel(config)}`);
  await pollStatus();
}

function stereoStatusText(frame) {
  if (!frame.available || !(Number(frame.seq) > 0)) {
    return "waiting";
  }
  const width = Number(frame.width) || 0;
  const height = Number(frame.height) || 0;
  const size = width > 0 && height > 0 ? `${width}x${height}` : "frame";
  const age = Number(frame.age_s);
  const ageText = Number.isFinite(age) ? `${fixed(age, 1)}s` : "n/a";
  return `${size}, ${ageText}`;
}

// Browser-only pacing. The camera topics, recorder and simulation clock are independent.
class PreviewRate {
  constructor() {
    this.setMode("auto");
  }

  setMode(value) {
    this.mode = ["auto", "0", "2", "4", "10"].includes(value) ? value : "auto";
    this.hz = this.mode === "auto" ? 4 : Number(this.mode);
    this.reset();
  }

  reset() {
    this.start = null;
    this.count = [0, 0];
    this.fps = [0, 0];
    this.slow = 0;
    this.maxLag = 0;
    this.healthyWindows = 0;
  }

  loaded(side, elapsedMs) {
    this.count[side === "left" ? 0 : 1]++;
    if (elapsedMs > 900 / this.hz) this.slow++;
  }

  sample(now, lagMs) {
    this.start ??= now;
    this.maxLag = Math.max(this.maxLag, lagMs);
    const elapsed = now - this.start;
    if (elapsed < 2000) return false;
    this.fps = this.count.map(n => n * 1000 / elapsed);
    const frames = this.count[0] + this.count[1];
    if (this.mode === "auto") {
      if (this.maxLag > 250 || (frames && this.slow >= frames / 4)) {
        this.hz = this.hz > 4 ? 4 : 2;
        this.healthyWindows = 0;
      } else if (Math.max(...this.fps) >= this.hz * 0.75) {
        if (++this.healthyWindows >= 3) {
          this.hz = this.hz === 2 ? 4 : 10;
          this.healthyWindows = 0;
        }
      } else {
        this.healthyWindows = 0;
      }
    }
    this.start = now;
    this.count = [0, 0];
    this.slow = 0;
    this.maxLag = 0;
    return true;
  }
}

const previewRate = new PreviewRate();
let lastPreviewTick = null;
let lastCameraPoll = -Infinity;
function previewVisible() {
  return !document.hidden && !$("stereoCameraPanel").closest("[hidden], .hidden");
}

function setPreviewMode(value) {
  previewRate.setMode(value);
  $("stereoCameraPreviewHz").value = previewRate.mode;
  lastPreviewTick = null;
  lastCameraPoll = -Infinity;
  state.previewLastFrame = {left: -Infinity, right: -Infinity};
  setText("previewReport", previewRate.hz ? tr("Waiting for preview measurement", "미리보기 측정 대기") : tr("Preview paused", "미리보기 일시 정지"));
}

function updateStereoView(side, frame) {
  const image = $(side === "right" ? "stereoRightImage" : "stereoLeftImage");
  const view = $(side === "right" ? "stereoRightView" : "stereoLeftView");
  if (!image || !view) {
    return;
  }
  const seq = Number(frame.seq) || 0;
  if (!frame.available || seq <= 0) {
    view.classList.add("no-signal");
    image.removeAttribute("src");
    state.stereoSeq[side] = 0;
    state.previewLastFrame[side] = -Infinity;
    return;
  }
  view.classList.remove("no-signal");
  const now = performance.now();
  if (!previewVisible() || previewRate.hz <= 0 || image.complete === false
      || now - state.previewLastFrame[side] < 1000 / previewRate.hz) return;
  if (state.stereoSeq[side] !== seq) {
    state.previewLastFrame[side] = now;
    state.stereoSeq[side] = seq;
    image.src = `/api/stereo/${side}.jpg?seq=${seq}`;
  }
}

function setStereoCameraExpanded(expanded) {
  const panel = $("stereoCameraPanel");
  const button = $("stereoCameraZoomBtn");
  if (!panel || !button) {
    return;
  }
  if (Boolean(expanded) === panel.classList.contains("expanded")) return;
  if (expanded) {
    state.cameraDock = {parent: panel.parentElement, next: panel.nextElementSibling};
    $("cameraWindowBody").append(panel);
  }
  panel.classList.toggle("expanded", Boolean(expanded));
  document.body.classList.toggle("camera-expanded", Boolean(expanded));
  setPilotControlDocked(Boolean(expanded) && window.innerWidth >= 900);
  if (expanded && state.pilotDock) $("cameraWindowBody").append($("pilotControlGroup"));
  button.textContent = expanded ? tr("Close expanded view · Esc", "확대 닫기 · Esc") : tr("Expand View", "크게 보기");
  if (expanded) {
    window.StationWindows.open("cameraDialog", button);
  } else {
    const dock = state.cameraDock;
    if (dock?.next?.parentElement === dock.parent) dock.parent.insertBefore(panel, dock.next);
    else if (dock?.parent) dock.parent.append(panel);
    state.cameraDock = null;
    window.StationWindows.close("cameraDialog");
  }
}

function setPilotControlDocked(docked) {
  const pilot = $("pilotControlGroup");
  if (!pilot) {
    return;
  }
  if (docked) {
    if (!state.pilotDock) {
      state.pilotDock = {
        parent: pilot.parentElement,
        next: pilot.nextElementSibling,
      };
    }
    pilot.classList.add("camera-pilot-docked");
    document.body.appendChild(pilot);
    return;
  }
  pilot.classList.remove("camera-pilot-docked");
  if (!state.pilotDock || !state.pilotDock.parent) {
    return;
  }
  if (state.pilotDock.next && state.pilotDock.next.parentElement === state.pilotDock.parent) {
    state.pilotDock.parent.insertBefore(pilot, state.pilotDock.next);
  } else {
    state.pilotDock.parent.appendChild(pilot);
  }
  state.pilotDock = null;
}

function toggleStereoCameraZoom() {
  const panel = $("stereoCameraPanel");
  if (!panel) {
    return;
  }
  setStereoCameraExpanded(!panel.classList.contains("expanded"));
}

function renderRcFeedback(telemetry) {
  if (!$("rcFeedbackBars").closest("details")?.open) return;
  const rcIn = Array.isArray(telemetry.rc_in) ? telemetry.rc_in : [];
  const rcOut = Array.isArray(telemetry.rc_out) ? telemetry.rc_out : [];
  const hasRcIn = rcIn.some((value) => Number(value) > 0);
  const channels = hasRcIn ? rcIn : rcOut;
  const source = hasRcIn ? telemetry.rc_in_source : telemetry.rc_out_source;
  setText("rcFeedbackSource", `feedback: ${source || "unavailable"}`);
  const container = $("rcFeedbackBars");
  container.replaceChildren();
  for (let index = 0; index < 8; index += 1) {
    const value = Number(channels[index]) || 0;
    const pct = value > 0 ? clamp((value - 1000) / 1000, 0, 1) * 100 : 0;
    const row = document.createElement("div");
    row.className = "rc-row";
    row.innerHTML = `<span>Ch ${String(index + 1).padStart(2, "0")}</span><div class="bar"><i style="width:${pct}%"></i></div><strong>${value}</strong>`;
    container.appendChild(row);
  }
}

function renderBlueRovMotors(telemetry) {
  const container = $("bluerovMotorRows");
  if (!container || document.body?.dataset?.vehicle !== "bluerov2_standard") return;
  const labels = [tr("Front right", "전방 우측"), tr("Front left", "전방 좌측"), tr("Rear right", "후방 우측"), tr("Rear left", "후방 좌측"), tr("Vertical right", "수직 우측"), tr("Vertical left", "수직 좌측")];
  const values = Array.isArray(telemetry.rc_out) ? telemetry.rc_out : [];
  const age = telemetryNumber(telemetry, "rc_out_age_s");
  const fresh = Number.isFinite(age) && age <= 2.5;
  const source = telemetry.rc_out_source || "unavailable";
  setText("bluerovMotorSource", fresh ? tr(`Output: ${source} · received ${age.toFixed(1)} s ago`, `출력: ${source} · ${age.toFixed(1)} s 전 수신`) : tr("Waiting for thruster output", "추진기 출력 수신 대기"));
  if (container.childElementCount !== labels.length) {
    container.replaceChildren();
    for (let index = 0; index < labels.length; index += 1) {
      const row = document.createElement("div");
      row.className = "bluerov-motor-row";
      const label = document.createElement("span");
      label.textContent = `T${index + 1} · ${labels[index]}`;
      const value = document.createElement("strong");
      row.append(label, value);
      container.appendChild(row);
    }
  }
  for (let index = 0; index < labels.length; index += 1) {
    const pwm = Number(values[index]);
    const valid = fresh && Number.isFinite(pwm) && pwm >= 1000 && pwm <= 2000;
    container.children[index].lastElementChild.textContent = valid ? `${Math.round(pwm)} μs` : "—";
  }
}

function renderEvents(events) {
  const list = $("eventList");
  if (!list.closest("details")?.open) return;
  const key = JSON.stringify(events.slice(0, 16));
  if (list.dataset.renderKey === key) return;
  list.dataset.renderKey = key;
  list.replaceChildren();
  events.slice(0, 16).forEach((text) => {
    const item = document.createElement("li");
    item.textContent = text;
    list.appendChild(item);
  });
}

function drawSensorUnavailable(canvas, label) {
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#08131d";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#86a4b7";
  ctx.font = "13px monospace";
  ctx.textAlign = "center";
  ctx.fillText(label, canvas.width / 2, canvas.height / 2);
  ctx.textAlign = "start";
}

function drawAttitude(telemetry) {
  const canvas = $("attitudeCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  if (!Number.isFinite(telemetryNumber(telemetry, "imu_age_s"))) {
    drawSensorUnavailable(canvas, "IMU / NO DATA");
    setText("attitudeText", "roll=— pitch=— yaw=—");
    return;
  }
  const roll = telemetryNumber(telemetry, "roll_deg", "roll");
  const pitch = telemetryNumber(telemetry, "pitch_deg", "pitch");
  const yaw = telemetryNumber(telemetry, "yaw_deg", "yaw");
  const displayRoll = Number.isFinite(roll) ? roll : 0;
  const displayPitch = Number.isFinite(pitch) ? pitch : 0;
  const displayYaw = Number.isFinite(yaw) ? yaw : 0;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#08131d";
  ctx.fillRect(0, 0, w, h);
  ctx.save();
  ctx.translate(w / 2, h / 2 + clamp(displayPitch / 45, -1, 1) * 38);
  ctx.rotate((displayRoll * Math.PI) / 180);
  ctx.fillStyle = "#183344";
  ctx.fillRect(-w, -h, w * 2, h);
  ctx.fillStyle = "#1c262c";
  ctx.fillRect(-w, 0, w * 2, h);
  ctx.strokeStyle = "#9abbce";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(-w, 0);
  ctx.lineTo(w, 0);
  ctx.stroke();
  ctx.restore();
  ctx.strokeStyle = "#c9e5f5";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(w / 2 - 42, h / 2);
  ctx.lineTo(w / 2 - 8, h / 2);
  ctx.moveTo(w / 2 + 8, h / 2);
  ctx.lineTo(w / 2 + 42, h / 2);
  ctx.moveTo(w / 2, h / 2 - 8);
  ctx.lineTo(w / 2, h / 2 + 8);
  ctx.stroke();
  ctx.fillStyle = "#c9e5f5";
  ctx.font = "13px system-ui";
  ctx.fillText(`yaw ${fixed(displayYaw, 2)}`, 12, 22);
  setText("attitudeText", `roll=${fixed(displayRoll, 2)} pitch=${fixed(displayPitch, 2)} yaw=${fixed(displayYaw, 2)}${telemetry.imu_age_s > 2 ? " · STALE" : ""}`);
}

function drawDepth(telemetry) {
  const canvas = $("depthCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const depth = telemetryNumber(telemetry, "depth_m", "depth");
  if (!Number.isFinite(depth)) {
    drawSensorUnavailable(canvas, "DEPTH / NO DATA");
    setText("depthText", "depth: — m");
    return;
  }
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#08131d";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#2b495d";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i += 1) {
    const y = 12 + (i / 5) * (h - 24);
    ctx.beginPath();
    ctx.moveTo(40, y);
    ctx.lineTo(w - 16, y);
    ctx.stroke();
  }
  const depthRatio = Number.isFinite(depth) ? clamp(depth / 20, 0, 1) : 0;
  const markerY = 12 + depthRatio * (h - 24);
  ctx.fillStyle = "#8acde9";
  ctx.beginPath();
  ctx.moveTo(28, markerY);
  ctx.lineTo(40, markerY - 7);
  ctx.lineTo(40, markerY + 7);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = "#9abbce";
  ctx.font = "13px system-ui";
  ctx.fillText(Number.isFinite(depth) ? `${fixed(depth, 3)} m` : "n/a", 48, markerY + 4);
  setText("depthText", Number.isFinite(depth) ? `depth: ${fixed(depth, 3)} m${telemetry.depth_age_s > 2 ? " · STALE" : ""}` : "depth: n/a");
}

async function pollStatus() {
  if (document.hidden || state.statusPollBusy) {
    return;
  }
  state.statusPollBusy = true;
  try {
    const response = await fetch("/api/status", { cache: "no-store" });
    if (response.ok) {
      renderStatus(await response.json());
    }
  } catch (error) {
    console.error(error);
  } finally {
    state.statusPollBusy = false;
  }
}

function pollArmTransition() {
  pollStatus();
  for (const delayMs of [100, 250, 450, 650, 850, 1100, 1400]) {
    window.setTimeout(pollStatus, delayMs);
  }
}

async function startPingerHoming() {
  if (state.pingerStartPending || $("pingerHomingStartBtn").disabled) {
    return;
  }
  state.pingerStartPending = true;
  $("pingerHomingStartBtn").disabled = true;
  $("pingerHomingStartBtn").textContent = "Pinger starting";
  releaseInput();
  setText("pingerHomingStatus", "pinger homing: starting");
  let runningAccepted = false;
  try {
    const result = await postCommand({ command: "pinger_homing_start", values: pingerHomingPayload() });
    runningAccepted = Boolean(result.running);
    if (runningAccepted) {
      $("pingerHomingStartBtn").disabled = true;
      $("pingerHomingStartBtn").textContent = "Pinger running";
    }
    await pollStatus();
  } catch (error) {
    setText("pingerHomingStatus", "pinger homing: start failed");
    console.error(error);
  } finally {
    state.pingerStartPending = false;
    // The next status render keeps this disabled when the process is running.
    // On an HTTP/start failure, immediately make retry possible.
    if (!runningAccepted) {
      $("pingerHomingStartBtn").disabled = false;
      $("pingerHomingStartBtn").textContent = "Start pinger";
    }
  }
}

async function requestArm(value) {
  setText("commandReady", value ? "ARM: request pending" : "DISARM: request pending");
  try {
    await postCommand({ command: "arm", value });
    pollArmTransition();
  } catch (error) {
    setText("commandReady", `${value ? "ARM" : "DISARM"}: request failed`);
    console.error(error);
  }
}

async function pollCameraStatus() {
  if (!previewVisible() || !state.stereoCameraEnabled || !previewRate.hz) {
    if (lastPreviewTick !== null) previewRate.reset();
    lastPreviewTick = null;
    const label = !previewRate.hz ? tr("Preview paused", "미리보기 일시 정지") :
      !state.stereoCameraEnabled ? tr("Camera off", "카메라 꺼짐") : tr("Open the camera view to resume the preview.", "영상 화면을 열면 미리보기가 재개됩니다.");
    setText("previewReport", label);
    return;
  }
  const now = performance.now();
  const lag = lastPreviewTick === null ? 0 : now - lastPreviewTick - CAMERA_POLL_MS;
  lastPreviewTick = now;
  if (previewRate.sample(now, lag)) {
    const [left, right] = previewRate.fps;
    const mode = previewRate.mode === "auto" ? tr("Auto", "자동") : tr("Manual", "수동");
    setText("previewReport", tr(`${mode} · cap ${previewRate.hz} fps · front ${left.toFixed(1)} / hand ${right.toFixed(1)} fps`, `${mode} · 상한 ${previewRate.hz} fps · 전방 ${left.toFixed(1)} / 손 ${right.toFixed(1)} fps`)
      + (left || right ? "" : tr(" · waiting for new frames", " · 새 영상 대기")));
  }
  if (state.cameraPollBusy || now - lastCameraPoll < 1000 / previewRate.hz - 5) return;
  lastCameraPoll = now;
  state.cameraPollBusy = true;
  try {
    const response = await fetch("/api/stereo/status", { cache: "no-store" });
    if (response.ok) {
      renderStereoCamera(await response.json());
    }
  } catch (error) {
    console.error(error);
  } finally {
    state.cameraPollBusy = false;
  }
}

function buildModeButtons() {
  const modes = ["MANUAL", "STABILIZE", "ALT_HOLD", "GUIDED", "SURFACE", "POSHOLD"];
  const container = $("modeButtons");
  modes.forEach((mode) => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = mode;
    button.addEventListener("click", () => postCommand({ command: "mode", mode }).catch(console.error));
    container.appendChild(button);
  });
}

function replayPayload() {
  if (document.body?.dataset?.vehicle === "bluerov2_standard" && !$("rcReplayPath").value.trim()) {
    setText("rcReplayStatus", tr("Enter the RC input rosbag path.", "RC 입력 rosbag 경로를 입력하세요."));
    return null;
  }
  return {
    path: $("rcReplayPath").value,
    rate: $("rcReplayRate").value,
  };
}

function pingerHomingPayload() {
  const numericValue = (id, fallback) => {
    const value = Number($(id).value);
    return Number.isFinite(value) ? value : fallback;
  };
  const homingMode = $("pingerHomingAlgorithm").value || "phase";
  return {
    algorithm: homingMode,
    navigation_mode: "odometry",
    transport: $("pingerHomingTransport").value,
    rate_hz: numericValue("pingerHomingRateHz", 30),
    forward_fast: numericValue("pingerHomingForwardFast", 0.48),
    probe_pwm_delta: numericValue("pingerHomingProbePwmDelta", 20),
    approach_pwm_delta: numericValue("pingerHomingApproachPwmDelta", 25),
    yaw_gain: numericValue("pingerHomingYawGain", 0.85),
    tank_max_depth_m: numericValue("pingerHomingTankMaxDepth", 11.0),
    success_range_m: numericValue("pingerHomingSuccessRange", 0.0),
    success_hold_s: numericValue("pingerHomingSuccessHold", 0.8),
    // Match the physical launch. The GUI requests this mode, and the C++
    // controller waits for fresh /mavros/state confirmation before RC output.
    mode: "ALT_HOLD",
    auto_arm: $("pingerHomingAutoArm").checked,
    auto_mode: false,
    // Standalone acoustic homing stops on calibrated acoustic range.  Vision
    // handoff belongs to the vision mission package, not this start button.
    use_yolo_final: false,
    stop_range_m: numericValue("pingerHomingSuccessRange", 0.0),
  };
}

function applyPingerHomingAlgorithmDefaults() {
  $("pingerHomingForwardFast").value = "0.48";
  $("pingerHomingYawGain").value = "0.85";
  $("pingerHomingProbePwmDelta").value = "20";
  $("pingerHomingApproachPwmDelta").value = "25";
  setText(
    "pingerHomingNavigationStatus",
    "PHASE_REAL_PARITY · 10 s FFT · /odometry/filtered · ALT_HOLD",
  );
}

async function openPhysicsDialog() {
  window.StationWindows.open("physicsDialog");
  setText("physicsDialogStatus", "physics params: loading");
  await postCommand({ command: "physics_params" });
  await loadPhysicsParams();
  await pollStatus();
}

async function loadPhysicsParams() {
  const body = await postCommand({ command: "physics_load" });
  state.physicsRows = Array.isArray(body.rows) ? body.rows : [];
  setText("physicsDialogPath", `path: ${body.path || "n/a"}`);
  setText("physicsDialogStatus", body.status || "physics params: loaded");
  renderPhysicsRows();
}

function renderPhysicsRows() {
  const container = $("physicsRows");
  container.replaceChildren();
  state.physicsRows.forEach((row) => {
    const item = document.createElement("div");
    item.className = row.inactive ? "physics-row inactive" : "physics-row";
    item.role = "row";

    const label = document.createElement("span");
    label.textContent = row.label || row.key;

    const input = document.createElement("input");
    input.className = "physics-value";
    input.setAttribute("aria-label", row.label || row.key);
    input.dataset.key = row.key;
    input.value = row.value ?? "";
    input.disabled = Boolean(row.inactive);
    input.title = row.inactive ? row.mode_status || "inactive" : row.description || "";
    if (row.kind === "scalar" && Number.isFinite(row.minimum) && Number.isFinite(row.maximum)) {
      input.type = "number";
      input.step = "any";
      input.min = String(row.minimum);
      input.max = String(row.maximum);
    }

    const key = document.createElement("code");
    key.textContent = row.key || "";

    const description = document.createElement("span");
    description.textContent = row.description || "";

    const mode = document.createElement("span");
    mode.className = row.inactive ? "mode-status inactive" : "mode-status";
    mode.textContent = row.mode_status || "active";

    item.append(label, input, key, description, mode);
    container.appendChild(item);
  });
}

function collectPhysicsValues() {
  const values = {};
  document.querySelectorAll(".physics-value").forEach((input) => {
    if (!input.disabled && input.dataset.key) {
      values[input.dataset.key] = input.value;
    }
  });
  return values;
}

async function applyPhysicsParams(restart = false) {
  setText("physicsDialogStatus", restart ? "physics params: applying + reset" : "physics params: applying");
  const body = await postCommand({ command: "physics_apply", values: collectPhysicsValues(), restart });
  state.physicsRows = Array.isArray(body.rows) ? body.rows : state.physicsRows;
  setText("physicsDialogPath", `path: ${body.path || "n/a"}`);
  setText("physicsDialogStatus", body.status || "physics params: applied");
  renderPhysicsRows();
  await pollStatus();
}

async function openCourseDialog() {
  window.StationWindows.open("courseDialog");
  setText("courseDialogStatus", "course layout: loading");
  await loadCourseLayout($("simLaunchPreset").value.startsWith("research_pool_") ? "research_pool" : "");
  await pollStatus();
}

async function loadCourseLayout(mode = "") {
  const body = await postCommand({ command: "course_load", mode });
  state.courseLayout = normalizeCourseLayout(body);
  if (!courseTargetById(state.selectedCourseTarget)) {
    state.selectedCourseTarget = state.courseLayout.robot?.id || state.courseLayout.items?.[0]?.id || "";
  }
  setText("courseDialogPath", `path: ${body.path || "n/a"}`);
  setText("courseDialogStatus", body.status || "course layout: loaded");
  renderCourseLayout();
}

function normalizeCourseLayout(body) {
  const robot = body.robot ? { ...body.robot, kind: "robot" } : null;
  const items = Array.isArray(body.items)
    ? body.items.map((item) => ({ ...item, kind: item.item_kind || "buoy" }))
    : [];
  return {
    path: body.path || "",
    status: body.status || "",
    tank: body.tank || { x_half_m: 17.5, y_half_m: 15.0 },
    mode: body.mode || "competition",
    activeMode: body.active_mode || body.mode || "competition",
    modeOptions: Array.isArray(body.mode_options) ? body.mode_options : [],
    robot,
    items,
  };
}

function courseTargets() {
  if (!state.courseLayout) {
    return [];
  }
  return [state.courseLayout.robot, ...state.courseLayout.items].filter(Boolean);
}

function courseTargetById(id) {
  return courseTargets().find((target) => target.id === id) || null;
}

function selectCourseTarget(id) {
  if (!courseTargetById(id)) {
    return;
  }
  state.selectedCourseTarget = id;
  renderCourseLayout();
}

function renderCourseLayout() {
  renderCourseMode();
  renderCourseRows();
  renderCourseCanvas();
  renderCourseSelection();
}

function syncCompetitionOption() {
  const option = $("competitionCourseOption");
  const show = $("legacyCourseEnabled").checked || state.courseLayout?.mode === "competition";
  option.hidden = !show;
  option.disabled = !show;
}

function renderCourseMode() {
  syncCompetitionOption();
  const layout = state.courseLayout;
  if (!layout) {
    return;
  }
  $("courseModeSelect").value = layout.mode;
  const tank = layout.tank || {};
  const length = Number(tank.length_m) || 2 * Number(tank.x_half_m || 0);
  const width = Number(tank.width_m) || 2 * Number(tank.y_half_m || 0);
  const depth = Number(tank.depth_m);
  const dimensions = Number.isFinite(depth)
    ? `${fixed(length, 2)} x ${fixed(width, 2)} x ${fixed(depth, 2)} m`
    : `${fixed(length, 2)} x ${fixed(width, 2)} m`;
  setText("courseTankDimensions", dimensions);
  setText("courseDepthHeading", layout.mode === "research_pool" ? tr("Depth ↓", "수심 ↓") : "Z");
  $("courseRawBtn").hidden = layout.mode === "research_pool";
}

function renderCourseRows() {
  const body = $("courseRows");
  body.replaceChildren();
  courseTargets().forEach((target) => {
    const row = document.createElement("tr");
    row.className = target.id === state.selectedCourseTarget ? "selected" : "";
    for (const text of [
      target.label || target.id,
      target.layer || "",
      fixed(target.x, 2),
      fixed(target.y, 2),
      fixed(state.courseLayout?.mode === "research_pool" ? -target.z : target.z, 2),
    ]) {
      const cell = document.createElement("td");
      cell.textContent = text;
      row.appendChild(cell);
    }
    row.addEventListener("click", () => selectCourseTarget(target.id));
    body.appendChild(row);
  });
}

function renderCourseSelection() {
  const target = courseTargetById(state.selectedCourseTarget);
  $("courseXInput").value = target ? fixed(target.x, 3) : "";
  $("courseYInput").value = target ? fixed(target.y, 3) : "";
  const research = state.courseLayout?.mode === "research_pool";
  $("courseZInput").readOnly = !research;
  $("courseZInput").value = target ? fixed(research ? -target.z : target.z, 3) : "";
  $("courseZInput").min = research ? "0.4" : "-100";
  $("courseZInput").max = research ? String(state.courseLayout.tank.depth_m - (target?.kind === "robot" ? .45 : .7)) : "100";
  setText("courseZLabel", research ? tr("Depth ↓", "수심 ↓") : tr("Z (fixed)", "Z (고정)"));
  $("courseLayerInput").value = target ? target.layer || "" : "";
}

function updateSelectedCourseFromInputs() {
  const target = courseTargetById(state.selectedCourseTarget);
  if (!target) {
    setText("courseDialogStatus", "course layout: select an item first");
    return false;
  }
  const x = Number($("courseXInput").value);
  const y = Number($("courseYInput").value);
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    setText("courseDialogStatus", "course layout: invalid X/Y number");
    return false;
  }
  if (state.courseLayout?.mode === "research_pool") {
    const depth = Number($("courseZInput").value);
    const maximum = state.courseLayout.tank.depth_m - (target.kind === "robot" ? .45 : .7);
    if (!Number.isFinite(depth) || depth < .4 || depth > maximum) {
      setText("courseDialogStatus", tr(`Enter a depth between 0.4 and ${maximum} m.`, `수심은 0.4 ~ ${maximum} m 범위로 입력하세요.`));
      return false;
    }
    target.z = -depth;
  }
  setCourseTargetPosition(target.id, x, y);
  renderCourseLayout();
  return true;
}

function setCourseTargetPosition(id, x, y) {
  const target = courseTargetById(id);
  if (!target || !state.courseLayout) {
    return;
  }
  const tank = state.courseLayout.tank;
  const margin = state.courseLayout.mode === "research_pool" ? (target.kind === "robot" ? .45 : .25) : 0;
  target.x = clamp(x, -Number(tank.x_half_m || 17.5) + margin, Number(tank.x_half_m || 17.5) - margin);
  target.y = clamp(y, -Number(tank.y_half_m || 15.0) + margin, Number(tank.y_half_m || 15.0) - margin);
}

function courseMetrics() {
  const canvas = $("courseCanvas");
  const tank = state.courseLayout?.tank || { x_half_m: 17.5, y_half_m: 15.0 };
  const width = canvas.width;
  const height = canvas.height;
  const xHalf = Number(tank.x_half_m) || 17.5;
  const yHalf = Number(tank.y_half_m) || 15.0;
  const scale = Math.min((width - 2 * COURSE_MARGIN) / (2 * xHalf), (height - 2 * COURSE_MARGIN) / (2 * yHalf));
  const tankW = 2 * xHalf * scale;
  const tankH = 2 * yHalf * scale;
  return {
    width,
    height,
    xHalf,
    yHalf,
    scale,
    left: (width - tankW) / 2,
    top: (height - tankH) / 2,
    right: (width + tankW) / 2,
    bottom: (height + tankH) / 2,
  };
}

function worldToCanvas(x, y, metrics = courseMetrics()) {
  return {
    x: metrics.left + (Number(x) + metrics.xHalf) * metrics.scale,
    y: metrics.top + (metrics.yHalf - Number(y)) * metrics.scale,
  };
}

function canvasToWorld(px, py, metrics = courseMetrics()) {
  return {
    x: clamp((px - metrics.left) / metrics.scale - metrics.xHalf, -metrics.xHalf, metrics.xHalf),
    y: clamp(metrics.yHalf - (py - metrics.top) / metrics.scale, -metrics.yHalf, metrics.yHalf),
  };
}

function canvasPoint(event) {
  const canvas = $("courseCanvas");
  const rect = canvas.getBoundingClientRect();
  return {
    x: ((event.clientX - rect.left) * canvas.width) / rect.width,
    y: ((event.clientY - rect.top) * canvas.height) / rect.height,
  };
}

function renderCourseCanvas() {
  const canvas = $("courseCanvas");
  const ctx = canvas.getContext("2d");
  const metrics = courseMetrics();
  ctx.clearRect(0, 0, metrics.width, metrics.height);
  ctx.fillStyle = "#0b1823";
  ctx.strokeStyle = "#0f172a";
  ctx.lineWidth = 2;
  ctx.fillRect(metrics.left, metrics.top, metrics.right - metrics.left, metrics.bottom - metrics.top);
  ctx.strokeRect(metrics.left, metrics.top, metrics.right - metrics.left, metrics.bottom - metrics.top);
  drawCourseGrid(ctx, metrics);
  if (state.courseLayout?.mode === "competition") {
    drawCourseMarks(ctx, metrics);
  }
  courseTargets().forEach((target) => drawCourseTarget(ctx, target, metrics));
}

function drawCourseGrid(ctx, metrics) {
  ctx.font = "11px system-ui";
  ctx.textBaseline = "top";
  ctx.strokeStyle = "#294354";
  ctx.fillStyle = "#91aabb";
  ctx.lineWidth = 1;
  const xStep = courseGridStep(metrics.xHalf * 2);
  const yStep = courseGridStep(metrics.yHalf * 2);
  for (let x = Math.ceil(-metrics.xHalf / xStep) * xStep; x <= metrics.xHalf + 1e-9; x += xStep) {
    const point = worldToCanvas(x, 0, metrics);
    ctx.beginPath();
    ctx.moveTo(point.x, metrics.top);
    ctx.lineTo(point.x, metrics.bottom);
    ctx.stroke();
    ctx.fillText(formatGridValue(x), point.x + 2, metrics.bottom + 8);
  }
  for (let y = Math.ceil(-metrics.yHalf / yStep) * yStep; y <= metrics.yHalf + 1e-9; y += yStep) {
    const point = worldToCanvas(0, y, metrics);
    ctx.beginPath();
    ctx.moveTo(metrics.left, point.y);
    ctx.lineTo(metrics.right, point.y);
    ctx.stroke();
    ctx.fillText(formatGridValue(y), metrics.left - 30, point.y - 6);
  }
  const center = worldToCanvas(0, 0, metrics);
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = "#64748b";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(center.x, metrics.top);
  ctx.lineTo(center.x, metrics.bottom);
  ctx.stroke();
  ctx.setLineDash([]);
}

function courseGridStep(span) {
  const target = Math.max(0.01, Number(span) / 7);
  const candidates = [0.1, 0.2, 0.25, 0.5, 1, 2, 5, 10];
  return candidates.find((candidate) => candidate >= target) || 10;
}

function formatGridValue(value) {
  const clean = Math.abs(value) < 1e-9 ? 0 : value;
  return Number.isInteger(clean) ? String(clean) : clean.toFixed(1);
}

function drawCourseMarks(ctx, metrics) {
  ctx.fillStyle = "#bdd5e4";
  ctx.font = "bold 14px system-ui";
  ctx.textAlign = "center";
  ctx.fillText("A course", (metrics.left + metrics.right) / 4, metrics.top + 16);
  ctx.fillText("B course", ((metrics.left + metrics.right) * 3) / 4, metrics.top + 16);
  ctx.strokeStyle = "#0284c7";
  ctx.lineWidth = 3;
  for (const gx of [-5.25, 5.25]) {
    const point = worldToCanvas(gx, 0, metrics);
    ctx.beginPath();
    ctx.arc(point.x, point.y, 0.75 * metrics.scale, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.textAlign = "start";
}

function drawCourseTarget(ctx, target, metrics) {
  const point = worldToCanvas(target.x, target.y, metrics);
  const selected = target.id === state.selectedCourseTarget;
  if (target.kind === "robot") {
    drawRobotTarget(ctx, target, point, selected);
    return;
  }
  if (target.kind === "pinger") {
    drawPingerTarget(ctx, target, point, selected);
    return;
  }
  const radius = COURSE_POINT_RADIUS + (selected ? 2 : 0);
  ctx.fillStyle = target.color_hex || "#64748b";
  ctx.strokeStyle = selected ? "#9ed8f4" : "#68879c";
  ctx.lineWidth = target.fixed_underwater ? 2 : 1;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  if (target.fixed_underwater) {
    ctx.strokeStyle = "#91aabb";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius + 4, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.fillStyle = "#bdd5e4";
  ctx.font = selected ? "bold 11px system-ui" : "11px system-ui";
  ctx.fillText(target.label || target.id, point.x + radius + 5, point.y + 4);
}

function drawPingerTarget(ctx, target, point, selected) {
  const radius = COURSE_POINT_RADIUS + (selected ? 3 : 1);
  ctx.strokeStyle = selected ? "#9ed8f4" : target.color_hex || "#dc2626";
  ctx.fillStyle = "#0b1823";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(point.x, point.y, Math.max(2, radius - 5), 0, Math.PI * 2);
  ctx.fillStyle = target.color_hex || "#dc2626";
  ctx.fill();
  ctx.strokeStyle = target.color_hex || "#dc2626";
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius + 5, -0.8, 0.8);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius + 9, -0.8, 0.8);
  ctx.stroke();
  ctx.fillStyle = "#bdd5e4";
  ctx.font = selected ? "bold 11px system-ui" : "11px system-ui";
  ctx.fillText(target.label || "Pinger", point.x + radius + 12, point.y + 4);
}

function drawRobotTarget(ctx, target, point, selected) {
  const radius = COURSE_POINT_RADIUS + 8 + (selected ? 3 : 0);
  ctx.fillStyle = target.color_hex || "#9ed8f4";
  ctx.strokeStyle = selected ? "#f97316" : "#bdd5e4";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(point.x + radius, point.y);
  ctx.lineTo(point.x - radius * 0.75, point.y - radius * 0.7);
  ctx.lineTo(point.x - radius * 0.35, point.y);
  ctx.lineTo(point.x - radius * 0.75, point.y + radius * 0.7);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = "#8acde9";
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius + 4, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#bdd5e4";
  ctx.font = "bold 12px system-ui";
  ctx.fillText(target.label || "ROBOT", point.x + radius + 7, point.y + 4);
}

function courseTargetAtPoint(px, py) {
  let best = null;
  let bestDistance = Infinity;
  const metrics = courseMetrics();
  for (const target of courseTargets()) {
    const point = worldToCanvas(target.x, target.y, metrics);
    const distance = Math.hypot(px - point.x, py - point.y);
    const hitRadius = target.kind === "robot" ? 24 : 18;
    if (distance <= hitRadius && distance < bestDistance) {
      best = target;
      bestDistance = distance;
    }
  }
  return best;
}

function bindCourseCanvas() {
  const canvas = $("courseCanvas");
  canvas.addEventListener("pointerdown", (event) => {
    const point = canvasPoint(event);
    const target = courseTargetAtPoint(point.x, point.y);
    if (!target) {
      return;
    }
    state.selectedCourseTarget = target.id;
    state.courseDragging = true;
    canvas.setPointerCapture(event.pointerId);
    renderCourseLayout();
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!state.courseDragging || !state.selectedCourseTarget) {
      return;
    }
    const point = canvasPoint(event);
    const world = canvasToWorld(point.x, point.y);
    setCourseTargetPosition(state.selectedCourseTarget, world.x, world.y);
    renderCourseLayout();
  });
  const finish = (event) => {
    if (!state.courseDragging) {
      return;
    }
    state.courseDragging = false;
    try {
      canvas.releasePointerCapture(event.pointerId);
    } catch (_error) {
      // Pointer may already be released by the browser.
    }
    renderCourseLayout();
  };
  canvas.addEventListener("pointerup", finish);
  canvas.addEventListener("pointercancel", finish);
}

function courseSavePayload() {
  if (!updateSelectedCourseFromInputs()) {
    return null;
  }
  const positions = {};
  if (state.courseLayout) {
    state.courseLayout.items.forEach((item) => {
      positions[item.id] = { x: item.x, y: item.y, ...(state.courseLayout.mode === "research_pool" ? { z: item.z } : {}) };
    });
  }
  const robot = state.courseLayout?.robot;
  return {
    mode: state.courseLayout?.mode || "competition",
    positions,
    robot_xy: robot ? { x: robot.x, y: robot.y, ...(state.courseLayout.mode === "research_pool" ? { z: robot.z } : {}) } : null,
  };
}

async function saveCourseLayout(reset = false) {
  const payload = courseSavePayload();
  if (!payload) {
    return;
  }
  setText("courseDialogStatus", reset ? "course layout: saving + reset" : "course layout: saving");
  let body;
  try {
    body = await postCommand({ command: "course_save", ...payload, reset });
  } catch (error) {
    setText("courseDialogStatus", tr(`Save failed: ${error.message}`, `저장 실패: ${error.message}`));
    return;
  }
  state.courseLayout = normalizeCourseLayout(body);
  setText("courseDialogPath", `path: ${body.path || "n/a"}`);
  setText("courseDialogStatus", body.status || "course layout: saved");
  renderCourseLayout();
  await pollStatus();
}

function bindControls() {
  for (const [id, command] of [
    ["vlaPrepareControlBtn", "vla_prepare_control"],
    ["vlaRestoreManualBtn", "vla_restore_manual"],
  ]) {
    $(id).addEventListener("click", async () => {
      try {
        releaseInput();
        const result = await postCommand({ command });
        state.vlaControlPrepared = Boolean(result.vla_prepared);
        setText("vlaControlStatus", result.status);
        await pollStatus();
      } catch (error) {
        setText("vlaControlStatus", error.message);
      }
    });
  }

  $("showAdvancedEnvironments").addEventListener("change", () => {
    renderSimulationConfig(state.lastSimulationConfig || {}, state.lastSimulationProcesses);
  });
  $("sensorErrorMode").addEventListener("change", () => {
    state.sensorErrorTouched = true;
    renderSimulationConfig(state.lastSimulationConfig || {}, state.lastSimulationProcesses);
  });

  $("armBtn").addEventListener("click", () => requestArm(true));
  $("disarmBtn").addEventListener("click", () => requestArm(false));
  $("releaseBtn").addEventListener("click", releaseInput);
  $("centerBtn").addEventListener("click", () => {
    centerAxes();
    sendRc(true);
  });
  $("quickCenterBtn").addEventListener("click", () => {
    centerAxes();
    sendRc(true);
  });
  $("rcEnabled").addEventListener("change", (event) => {
    state.rcEnabled = event.target.checked;
    sendRc(true);
  });
  $("gamepadEnabled").addEventListener("change", () => {
    // Disabling the checkbox while a stick is held sends a neutral/release
    // frame immediately; enabling it is then picked up by the 50 Hz poll.
    pollPhysicalGamepad();
  });
  $("gamepadDevice").addEventListener("change", (event) => {
    releaseInput();
    state.gamepadIndex = event.target.value === "" ? null : Number(event.target.value);
    pollPhysicalGamepad();
  });
  window.addEventListener("gamepadconnected", (event) => {
    renderGamepadStatus(`Gamepad: ${gamepadLabel(event.gamepad)} — connected`);
    pollPhysicalGamepad();
  });
  window.addEventListener("gamepaddisconnected", (event) => {
    if (event.gamepad.index === state.gamepadIndex) {
      releasePhysicalGamepad("disconnected");
    }
  });
  for (const id of ["axisForward", "axisLateral", "axisHeave", "axisYaw"]) {
    $(id).addEventListener("input", () => {
      readAxesFromSliders();
      sendRc(false);
    });
    $(id).addEventListener("change", () => sendRc(true));
  }
  bindJoystick($("leftStick"), { x: "yaw", y: "heave" });
  bindJoystick($("rightStick"), { x: "lateral", y: "forward" });

  $("telemetryToggle").addEventListener("click", () => {
    document.body.classList.toggle("telemetry-hidden");
    $("telemetryToggle").textContent = document.body.classList.contains("telemetry-hidden")
      ? tr("Show Telemetry", "상태 패널 펼치기")
      : tr("Hide Telemetry", "상태 패널 접기");
    $("telemetryToggle").setAttribute("aria-expanded", String(!document.body.classList.contains("telemetry-hidden")));
  });
  $("vehicleDetailsToggle").addEventListener("click", () =>
    setToggle("vehicleDetails", "vehicleDetailsToggle", "Details >", "Details <")
  );
  $("controlDetailsToggle").addEventListener("click", () =>
    setToggle("controlDetails", "controlDetailsToggle", "Details >", "Details <")
  );
  $("ros2PanelToggle").addEventListener("click", () =>
    setToggle("ros2Panel", "ros2PanelToggle", "ROS2 Panel", "Hide ROS2")
  );
  $("toolsToggle").addEventListener("click", () => setToggle("toolsBody", "toolsToggle", tr("Show Experiment Tools", "실험 도구 펼치기"), tr("Hide Experiment Tools", "실험 도구 접기")));
  $("stereoCameraEnabled").addEventListener("change", (event) => {
    state.stereoCameraEnabled = event.target.checked;
    postCommand({ command: "stereo_camera_enabled", enabled: state.stereoCameraEnabled })
      .then(pollStatus)
      .catch(console.error);
  });
  $("stereoCameraVisionEnabled").addEventListener("change", (event) => {
    state.stereoCameraVisionEnabled = event.target.checked;
    postCommand({ command: "vision_processing_enabled", enabled: state.stereoCameraVisionEnabled })
      .then(pollStatus)
      .catch(console.error);
  });
  $("stereoCameraProfile").addEventListener("change", () => {
    setText("stereoCameraConfigStatus", `profile: ${selectedCameraPresetLabel()} selected`);
  });
  let mode = "auto";
  try { mode = localStorage.getItem("uuv.previewMode") || mode; } catch { /* storage unavailable */ }
  setPreviewMode(mode);
  for (const side of ["left", "right"]) {
    const image = $(side === "left" ? "stereoLeftImage" : "stereoRightImage");
    image.addEventListener("load", () => {
      if (previewRate.hz && previewVisible() && Number.isFinite(state.previewLastFrame[side])) {
        previewRate.loaded(side, performance.now() - state.previewLastFrame[side]);
      }
    });
  }
  $("stereoCameraPreviewHz").addEventListener("change", (event) => {
    setPreviewMode(event.target.value);
    try { localStorage.setItem("uuv.previewMode", previewRate.mode); } catch { /* storage unavailable */ }
    pollCameraStatus();
  });
  $("stereoCameraApplyBtn").addEventListener("click", () => applyCameraConfig(true).catch(console.error));
  $("stereoCameraSaveBtn").addEventListener("click", () => applyCameraConfig(false).catch(console.error));
  $("stereoCameraZoomBtn").addEventListener("click", toggleStereoCameraZoom);
  $("cameraDialog").addEventListener("station-window-closed", () => setStereoCameraExpanded(false));
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      setStereoCameraExpanded(false);
    }
  });

  $("simVehicle").addEventListener("change", (event) => {
    if (event.target.value.startsWith("real2sim:")) {
      state.real2simChoice = event.target.value.slice("real2sim:".length);
      renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
      document.querySelector('.station-nav [aria-controls="station-setup"]')?.click();
      return;
    }
    state.real2simChoice = "";
    const preset = state.lastSimulationConfig?.presets?.find(entry => entry.vehicle === event.target.value);
    if (preset) {
      state.simPresetChoice = preset.id;
      state.simStartError = "";
      renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
    }
  });
  $("simLaunchPreset").addEventListener("change", (event) => {
    state.simPresetChoice = event.target.value;
    state.simStartError = "";
    renderSimulationConfig(state.lastSimulationConfig, state.lastSimulationProcesses);
  });
  $("real2simViewBtn").addEventListener("click", openReal2SimReplay);
  $("real2simStopBtn").addEventListener("click", stopReal2SimReplay);
  $("real2simRefreshBtn").addEventListener("click", refreshReal2SimVehicles);
  $("stackStartBtn").addEventListener("click", startSimStack);
  $("stackStopBtn").addEventListener("click", () => stopSimStack().catch(console.error));
  $("stackResetBtn").addEventListener("click", () => postCommand({ command: "stack_reset" }).then(pollStatus).catch(console.error));
  $("rosBuildBtn").addEventListener("click", () => postCommand({ command: "ros_build" }).then(pollStatus).catch(console.error));
  $("mavrosToggleBtn").addEventListener("click", () =>
    postCommand({ command: "mavros_toggle", fcu_url: $("fcuUrlInput").value }).then(pollStatus).catch(console.error)
  );
  $("rvizToggleBtn").addEventListener("click", () => postCommand({ command: "rviz_toggle" }).then(pollStatus).catch(console.error));
  $("pingerHomingStartBtn").addEventListener("click", startPingerHoming);
  $("pingerHomingAlgorithm").addEventListener("change", applyPingerHomingAlgorithmDefaults);
  $("pingerHomingStopBtn").addEventListener("click", () => {
    setText("pingerHomingStatus", "pinger homing: stopping");
    postCommand({ command: "pinger_homing_stop" }).then(pollStatus).catch(console.error);
  });
  $("rcReplayLoadBtn").addEventListener("click", () => {
    const values = replayPayload();
    if (values) postCommand({ command: "rc_replay_load", ...values }).then(pollStatus).catch(console.error);
  });
  $("rcReplayPlayBtn").addEventListener("click", () => {
    const values = replayPayload();
    if (values) postCommand({ command: "rc_replay_play", ...values }).then(pollStatus).catch(console.error);
  });
  $("rcReplayPauseBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_pause" }).then(pollStatus).catch(console.error)
  );
  $("rcReplayStopBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_stop" }).then(pollStatus).catch(console.error)
  );
  $("rcReplaySlider").addEventListener("change", (event) =>
    postCommand({ command: "rc_replay_seek", time_s: Number(event.target.value) || 0 }).then(pollStatus).catch(console.error)
  );

  $("physicsOpenBtn").addEventListener("click", () => openPhysicsDialog().catch(error => setText("physicsDialogStatus", tr(`Load failed: ${error.message}`, `불러오기 실패: ${error.message}`))));
  $("courseOpenBtn").addEventListener("click", () => openCourseDialog().catch(error => setText("courseDialogStatus", tr(`Load failed: ${error.message}`, `불러오기 실패: ${error.message}`))));
  $("physicsCloseBtn").addEventListener("click", () => window.StationWindows.close("physicsDialog"));
  $("physicsReloadBtn").addEventListener("click", () => loadPhysicsParams().catch(console.error));
  $("physicsApplyBtn").addEventListener("click", () => applyPhysicsParams(false).catch(console.error));
  $("physicsApplyRestartBtn").addEventListener("click", () => applyPhysicsParams(true).catch(console.error));
  $("physicsRawBtn").addEventListener("click", () => openToolEditor("physics").catch(console.error));
  $("courseCloseBtn").addEventListener("click", () => window.StationWindows.close("courseDialog"));
  $("courseReloadBtn").addEventListener("click", () => loadCourseLayout($("courseModeSelect").value).catch(console.error));
  $("legacyCourseEnabled").addEventListener("change", syncCompetitionOption);
  $("courseModeSelect").addEventListener("change", (event) => loadCourseLayout(event.target.value).catch(console.error));
  $("courseUpdateSelectedBtn").addEventListener("click", updateSelectedCourseFromInputs);
  $("courseSaveBtn").addEventListener("click", () => saveCourseLayout(false).catch(console.error));
  $("courseSaveResetBtn").addEventListener("click", () => saveCourseLayout(true).catch(console.error));
  $("courseRawBtn").addEventListener("click", () => openToolEditor("course").catch(console.error));
  $("toolEditorCloseBtn").addEventListener("click", () => window.StationWindows.close("toolEditorDialog"));
  $("toolEditorReloadBtn").addEventListener("click", () => loadToolFile(state.toolKind).catch(console.error));
  $("toolEditorSaveBtn").addEventListener("click", () => saveToolFile().catch(console.error));

  $("ping360PanelBtn").addEventListener("click", () => window.StationWindows.open("ping360Dialog"));
  $("ping360CloseBtn").addEventListener("click", () => window.StationWindows.close("ping360Dialog"));
  $("pingEnableBtn").addEventListener("click", () =>
    postCommand({ command: "ping360_enabled", enabled: $("pingEnabled").checked }).then(pollStatus).catch(console.error)
  );
  $("pingConfigBtn").addEventListener("click", () => postCommand(pingConfigPayload()).then(pollStatus).catch(console.error));
  $("pingViewStartBtn").addEventListener("click", () =>
    postCommand({ command: "ping360_view_start" }).then(pollStatus).catch(console.error)
  );
  $("pingViewStopBtn").addEventListener("click", () =>
    postCommand({ command: "ping360_view_stop" }).then(pollStatus).catch(console.error)
  );
}

function pingConfigPayload() {
  return {
    command: "ping360_config",
    range_m: Number($("pingRange").value) || 2.0,
    num_steps: Number($("pingSteps").value) || 1,
    gain: Number($("pingGain").value) || 0,
    interface_mode: $("pingInterface").value || "ethernet",
    frequency_khz: Number($("pingFrequency").value) || 750,
    start_angle_grad: Number($("pingStartAngle").value) || 0,
    stop_angle_grad: Number($("pingStopAngle").value) || 399,
  };
}

async function openToolEditor(kind) {
  state.toolKind = kind;
  window.StationWindows.open("toolEditorDialog");
  $("toolEditorTitle").textContent = kind === "physics" ? "Physics Params" : "XY Course Layout";
  setText("toolEditorStatus", "loading");
  await postCommand({ command: kind === "physics" ? "physics_params" : "course_layout" });
  await loadToolFile(kind);
  await pollStatus();
}

async function loadToolFile(kind) {
  if (!kind) {
    return;
  }
  const body = await postCommand({ command: "tool_read", kind });
  setText("toolEditorPath", `path: ${body.path || "n/a"}`);
  setText("toolEditorStatus", "loaded");
  $("toolEditorText").value = body.content || "";
}

async function saveToolFile() {
  if (!state.toolKind) {
    return;
  }
  const body = await postCommand({
    command: "tool_save",
    kind: state.toolKind,
    content: $("toolEditorText").value,
  });
  setText("toolEditorStatus", `saved, backup: ${body.backup_path || "n/a"}`);
  await pollStatus();
}

buildModeButtons();
bindControls();
bindCourseCanvas();
centerAxes({ send: false });
pollPhysicalGamepad();
document.addEventListener("visibilitychange", () => {
  if (document.hidden) {
    releaseRcForInactivePage();
    return;
  }
  state.pageReleaseSent = false;
  pollStatus();
  pollCameraStatus();
});
window.addEventListener("pagehide", releaseRcForInactivePage);
pollStatus();
refreshReal2SimVehicles();
setInterval(refreshReal2SimVehicles, 15000);
setInterval(pollStatus, STATUS_POLL_MS);
setInterval(pollCameraStatus, CAMERA_POLL_MS);
setInterval(pollPhysicalGamepad, GAMEPAD_POLL_MS);
setInterval(() => {
  // Neutral is a live pilot command too. Stopping its heartbeat makes the
  // server watchdog release RC whenever the operator centres the sticks.
  if (state.rcEnabled) {
    sendRc(true);
  }
}, RC_KEEPALIVE_MS);
