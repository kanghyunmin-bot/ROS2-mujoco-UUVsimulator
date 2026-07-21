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
  stereoSeq: { left: 0 },
  cameraPollBusy: false,
  cameraPresetSignature: "",
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

function fixed(value, digits = 2) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "n/a";
}

function telemetryNumber(telemetry, ...keys) {
  for (const key of keys) {
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

function applyGamepadDeadzone(value) {
  const axis = clamp(value);
  const magnitude = Math.abs(axis);
  if (magnitude <= GAMEPAD_DEADZONE) {
    return 0;
  }
  // Rescale after the deadzone so full stick range remains available.
  return Math.sign(axis) * (magnitude - GAMEPAD_DEADZONE) / (1 - GAMEPAD_DEADZONE);
}

function physicalGamepadAxes(gamepad) {
  const axis = (index) => applyGamepadDeadzone(gamepad.axes?.[index] ?? 0);
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
  const pads = Array.from(navigator.getGamepads()).filter(Boolean);
  if (!pads.length) {
    state.gamepadIndex = null;
    return null;
  }
  const selected = pads.find((pad) => pad.index === state.gamepadIndex) || pads[0];
  state.gamepadIndex = selected.index;
  return selected;
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
  const response = await fetch("/api/rc", {
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

function setText(id, value) {
  const el = $(id);
  if (el) {
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
      renderGamepadStatus("Gamepad: connect controller, then move a stick");
    }
    return;
  }

  const axes = physicalGamepadAxes(gamepad);
  const active = Object.values(axes).some((value) => Math.abs(value) > 0.001);
  renderGamepadStatus(
    active
      ? `Gamepad: ${gamepadLabel(gamepad)} — RC active`
      : `Gamepad: ${gamepadLabel(gamepad)} — centered`
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
}

function renderStatus(payload) {
  const telemetry = payload.telemetry || {};
  const ui = payload.ui || {};
  const processes = payload.processes || {};
  const control = payload.control || {};
  const tools = payload.tools || {};

  if (typeof control.enabled === "boolean") {
    state.rcEnabled = Boolean(control.enabled);
    const rcToggle = $("rcEnabled");
    if (rcToggle && document.activeElement !== rcToggle) {
      rcToggle.checked = state.rcEnabled;
    }
  }

  setText("simStackPill", ui.sim_stack_status || "sim: stopped");
  setText("rosPkgPill", ui.ros_pkg_status || "mavros: stopped");
  setText("ping360Pill", ui.ping360_summary || "ping360: no status");
  setText("vehicleSummary", ui.vehicle_summary || "vehicle: disconnected");
  setText("motionSummary", ui.motion_summary || "motion: n/a");
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
  setText("simStackStatus", ui.sim_stack_status || "sim: stopped");
  setText("rosPkgStatus", ui.ros_pkg_status || "mavros: stopped");
  setText("rvizStatus", ui.rviz_status || "rviz: stopped");
  setText("missionFsmStatus", ui.mission_status || processes.mission_status || "mission: stopped");
  setText("pingerHomingStatus", ui.pinger_homing_status || processes.pinger_homing_status || "pinger homing: stopped");
  setText("ping360Summary", ui.ping360_summary || "ping360: no status");
  setText("ping360ViewStatus", ui.ping360_view_status || "ping360 view: closed");
  setText("rcReplayStatus", tools.rc_replay_status || "replay: unloaded");
  setText("rcReplayTime", tools.rc_replay_time || "00:00.0 / 00:00.0");
  setText("physicsStatus", tools.physics_status || "physics params: idle");
  setText("courseStatus", tools.buoy_layout_status || "course layout: idle");
  setText("physicsPath", `path: ${tools.physics_profile_path || "loading"}`);
  setText("coursePath", `path: ${tools.course_scene_path || "loading"}`);
  renderStep("attitude", () => drawAttitude(telemetry));
  renderStep("depth", () => drawDepth(telemetry));
  renderStep("camera config", () => renderCameraConfig(payload.camera_config || processes.camera_config || {}));
  renderStep("stereo camera", () => renderStereoCamera(payload.stereo_camera || {}));
  renderStep("mission monitor", () => renderMissionMonitor(payload.mission_monitor || processes.mission_monitor || {}));

  setValue("rcReplayPath", tools.rc_replay_path || "");
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
  pingerStartButton.disabled = pingerRunning || state.pingerStartPending;

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
    setText("stereoCameraStatus", "camera: off");
    return;
  }
  updateStereoView("left", frame);
  const errorText = camera.error ? ` | ${camera.error}` : "";
  const source = camera.display_source === "vision" ? "vision" : "raw";
  const fallback = camera.vision_fallback ? " (waiting for overlay)" : "";
  setText(
    "stereoCameraStatus",
    `camera: ${stereoStatusText(frame)} | view: ${source}${fallback}${yoloStatusText(camera.detection)}${errorText}`
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
  const signature = presets
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
  setText("stereoCameraConfigStatus", `profile: ${cameraConfigLabel(config)}`);
}

function cameraConfigLabel(config) {
  if (config && config.label) {
    return String(config.label);
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

function updateStereoView(side, frame) {
  const image = $("stereoLeftImage");
  const view = $("stereoLeftView");
  if (!image || !view) {
    return;
  }
  const seq = Number(frame.seq) || 0;
  if (!frame.available || seq <= 0) {
    view.classList.add("no-signal");
    image.removeAttribute("src");
    state.stereoSeq[side] = 0;
    return;
  }
  view.classList.remove("no-signal");
  if (state.stereoSeq[side] !== seq) {
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
  panel.classList.toggle("expanded", Boolean(expanded));
  document.body.classList.toggle("camera-expanded", Boolean(expanded));
  setPilotControlDocked(Boolean(expanded));
  button.textContent = expanded ? "Close" : "Expand";
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

function renderEvents(events) {
  const list = $("eventList");
  list.replaceChildren();
  events.slice(0, 16).forEach((text) => {
    const item = document.createElement("li");
    item.textContent = text;
    list.appendChild(item);
  });
}

function renderMissionMonitor(mission) {
  const processText = mission.process_status || "mission: stopped";
  const age = Number(mission.status_age_s);
  const ageText = Number.isFinite(age) && mission.available ? `, age ${fixed(age, 1)}s` : "";
  setText("missionMonitorProcess", `${processText}${ageText}`);
  if (!mission.available) {
    setText("missionMonitorState", mission.running ? "WAIT_DATA" : "stopped");
    setText("missionMonitorElapsed", "n/a");
    setText("missionMonitorRobotState", "n/a");
    setText("missionMonitorMode", "n/a");
    setText("missionMonitorTarget", "n/a");
    setText("missionMonitorCapture", "n/a");
    setText("missionMonitorCollectorEq", "n/a");
    setText("missionMonitorCounts", "n/a");
    setText("missionMonitorIntake", "n/a");
    setText("missionMonitorCommand", "n/a");
    setText("missionMonitorRobot", "n/a");
    renderMissionBuoys([]);
    return;
  }
  const displayState = mission.waiting_for_pose ? "WAIT_POSE" : mission.waiting_for_arm ? "WAIT_ARM" : mission.state || "n/a";
  setText("missionMonitorState", displayState);
  const elapsed = Number(mission.mission_elapsed_s);
  setText("missionMonitorElapsed", Number.isFinite(elapsed) ? `${fixed(elapsed, 1)}s` : "n/a");
  setText("missionMonitorRobotState", mission.robot_state_label || mission.robot_state || "n/a");
  const armedText = mission.armed === true ? "armed" : mission.armed === false ? "disarmed" : "arm n/a";
  setText("missionMonitorMode", `${mission.mode || "n/a"} | ${armedText}`);
  const targetClass = mission.target_class || mission.target_label || "";
  const target = mission.target_id || mission.collector_target_id
    ? `${targetClass} ${mission.target_id || mission.collector_target_id} ${mission.target_state || ""}`.trim()
    : targetClass || "none";
  setText("missionMonitorTarget", target);
  setText("missionMonitorCapture", mission.capture_state || (mission.capture_flag ? "CAPTURED" : "FREE"));
  setText("missionMonitorCollectorEq", mission.collector_eq_active === true ? "active" : "inactive");
  setText(
    "missionMonitorCounts",
    `rem ${mission.remaining_attached ?? 0} | detach ${mission.detached_count ?? mission.processed_count ?? 0} | net ${
      mission.netted_count ?? mission.collected_count ?? 0
    } | release ${mission.released_count ?? mission.scored_count ?? 0} | fail ${mission.failed_count ?? 0}`
  );
  const p = Array.isArray(mission.detection?.p_intake) ? mission.detection.p_intake : null;
  const source = mission.detection?.coordinate_source || "";
  setText(
    "missionMonitorIntake",
    p ? `${fixed(p[0], 2)}, ${fixed(p[1], 2)}, ${fixed(p[2], 2)}${source ? ` (${source})` : ""}` : "n/a"
  );
  const cmd = mission.command || {};
  setText(
    "missionMonitorCommand",
    `${fixed(cmd.forward)} ${fixed(cmd.sway)} ${fixed(cmd.heave)} ${fixed(cmd.yaw)}${cmd.phase ? ` ${cmd.phase}` : ""}`
  );
  const robot = mission.robot || {};
  if ((!Array.isArray(robot.xyz) || robot.xyz.length < 3) && Array.isArray(mission.robot_xyz)) {
    [robot.x, robot.y, robot.z] = mission.robot_xyz;
    robot.depth_m = -Number(robot.z);
  }
  const depth = Number(robot.depth_m);
  const yaw = Number(robot.yaw_rad);
  const x = Number(robot.x);
  const y = Number(robot.y);
  const z = Number(robot.z);
  const xyzText = [x, y, z].every(Number.isFinite)
    ? `${fixed(x, 2)}, ${fixed(y, 2)}, ${fixed(z, 2)}`
    : "xyz n/a";
  setText(
    "missionMonitorRobot",
    `${xyzText} | depth ${Number.isFinite(depth) ? `${fixed(depth, 2)}m` : "n/a"} | yaw ${Number.isFinite(yaw) ? fixed(yaw, 2) : "n/a"}`
  );
  renderMissionBuoys(Array.isArray(mission.buoys) ? mission.buoys : [], mission.target_id || "");
}

function renderMissionBuoys(buoys, targetId = "") {
  const body = $("missionBuoyRows");
  body.replaceChildren();
  if (!buoys.length) {
    const row = document.createElement("tr");
    row.innerHTML = '<td colspan="7">no mission data</td>';
    body.appendChild(row);
    return;
  }
  buoys.forEach((buoy) => {
    const row = document.createElement("tr");
    if (buoy.id === targetId) {
      row.className = "current-target";
    }
    const xyz = Array.isArray(buoy.target_xyz) ? buoy.target_xyz : Array.isArray(buoy.xyz) ? buoy.xyz : [];
    const source = buoy.coordinate_source || "";
    const flags = [
      source,
      buoy.physical_detached ? "detached" : "",
      buoy.eq_active === false ? "eq off" : "",
      buoy.processed ? "processed" : "",
      buoy.failed ? "failed" : "",
    ].filter(Boolean).join(", ");
    const stateClass = String(buoy.state || "").toLowerCase();
    const cells = [
      buoy.id || "",
      buoy.course || "",
      buoy.class_name || "",
      `<span class="mission-state ${stateClass}">${buoy.state || ""}</span>`,
      xyz.length >= 3 ? `${fixed(xyz[0], 1)}, ${fixed(xyz[1], 1)}, ${fixed(xyz[2], 1)}` : "n/a",
      `${fixed(buoy.release_force_threshold_n, 1)}N`,
      flags || "-",
    ];
    cells.forEach((value, index) => {
      const cell = document.createElement("td");
      if (index === 3) {
        cell.innerHTML = value;
      } else {
        cell.textContent = value;
      }
      row.appendChild(cell);
    });
    body.appendChild(row);
  });
}

function drawAttitude(telemetry) {
  const canvas = $("attitudeCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const roll = telemetryNumber(telemetry, "roll_deg", "roll");
  const pitch = telemetryNumber(telemetry, "pitch_deg", "pitch");
  const yaw = telemetryNumber(telemetry, "yaw_deg", "yaw");
  const displayRoll = Number.isFinite(roll) ? roll : 0;
  const displayPitch = Number.isFinite(pitch) ? pitch : 0;
  const displayYaw = Number.isFinite(yaw) ? yaw : 0;
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#0f172a";
  ctx.fillRect(0, 0, w, h);
  ctx.save();
  ctx.translate(w / 2, h / 2 + clamp(displayPitch / 45, -1, 1) * 38);
  ctx.rotate((displayRoll * Math.PI) / 180);
  ctx.fillStyle = "#1d4ed8";
  ctx.fillRect(-w, -h, w * 2, h);
  ctx.fillStyle = "#7c4a22";
  ctx.fillRect(-w, 0, w * 2, h);
  ctx.strokeStyle = "#e5e7eb";
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(-w, 0);
  ctx.lineTo(w, 0);
  ctx.stroke();
  ctx.restore();
  ctx.strokeStyle = "#f8fafc";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(w / 2 - 42, h / 2);
  ctx.lineTo(w / 2 - 8, h / 2);
  ctx.moveTo(w / 2 + 8, h / 2);
  ctx.lineTo(w / 2 + 42, h / 2);
  ctx.moveTo(w / 2, h / 2 - 8);
  ctx.lineTo(w / 2, h / 2 + 8);
  ctx.stroke();
  ctx.fillStyle = "#f8fafc";
  ctx.font = "13px system-ui";
  ctx.fillText(`yaw ${fixed(displayYaw, 2)}`, 12, 22);
  setText("attitudeText", `roll=${fixed(displayRoll, 2)} pitch=${fixed(displayPitch, 2)} yaw=${fixed(displayYaw, 2)}`);
}

function drawDepth(telemetry) {
  const canvas = $("depthCanvas");
  const ctx = canvas.getContext("2d");
  const w = canvas.width;
  const h = canvas.height;
  const depth = telemetryNumber(telemetry, "depth_m", "depth");
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = "#081018";
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#1f8a70";
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
  ctx.fillStyle = "#eab308";
  ctx.beginPath();
  ctx.moveTo(28, markerY);
  ctx.lineTo(40, markerY - 7);
  ctx.lineTo(40, markerY + 7);
  ctx.closePath();
  ctx.fill();
  ctx.fillStyle = "#e5e7eb";
  ctx.font = "13px system-ui";
  ctx.fillText(Number.isFinite(depth) ? `${fixed(depth, 3)} m` : "n/a", 48, markerY + 4);
  setText("depthText", Number.isFinite(depth) ? `depth: ${fixed(depth, 3)} m` : "depth: n/a");
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
  if (document.hidden || !state.stereoCameraEnabled || state.cameraPollBusy) {
    return;
  }
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
  return {
    path: $("rcReplayPath").value,
    rate: $("rcReplayRate").value,
  };
}

function missionPayload() {
  const course = $("missionCourse").value;
  const ownCourse = $("missionOwnCourse").value === "b" ? "b" : "a";
  const rawMaxTargets = Number($("missionMaxTargets").value) || 0;
  return {
    course,
    own_course: course === "all" ? ownCourse : course,
    max_targets: course === "all" && rawMaxTargets <= 1 ? 0 : rawMaxTargets,
    rate_hz: Number($("missionRateHz").value) || 30,
    transport: $("missionTransport").value,
    no_pinger: course === "all" ? false : $("missionNoPinger").checked,
    nearest_first: course === "all" ? true : $("missionNearestFirst").checked,
    dry_run: $("missionDryRun").checked,
    mission_log: "auto",
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

function applyMissionCourseDefaults() {
  const course = $("missionCourse").value;
  const allCourse = course === "all";
  $("missionOwnCourse").disabled = !allCourse;
  if (!allCourse) {
    $("missionOwnCourse").value = course;
  }
  if (allCourse) {
    if ((Number($("missionMaxTargets").value) || 0) <= 1) {
      $("missionMaxTargets").value = "0";
    }
    $("missionNoPinger").checked = false;
    $("missionNearestFirst").checked = true;
  }
}

async function openPhysicsDialog() {
  $("physicsDialog").classList.remove("hidden");
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
  $("courseDialog").classList.remove("hidden");
  setText("courseDialogStatus", "course layout: loading");
  await postCommand({ command: "course_layout" });
  await loadCourseLayout();
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

function renderCourseMode() {
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
      fixed(target.z, 2),
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
  $("courseZInput").value = target ? fixed(target.z, 3) : "";
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
  target.x = clamp(x, -Number(tank.x_half_m || 17.5), Number(tank.x_half_m || 17.5));
  target.y = clamp(y, -Number(tank.y_half_m || 15.0), Number(tank.y_half_m || 15.0));
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
  ctx.fillStyle = "#dff8ff";
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
  ctx.strokeStyle = "#b7dce8";
  ctx.fillStyle = "#475569";
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
  ctx.fillStyle = "#0f172a";
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
  ctx.strokeStyle = selected ? "#2563eb" : "#020617";
  ctx.lineWidth = target.fixed_underwater ? 2 : 1;
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  if (target.fixed_underwater) {
    ctx.strokeStyle = "#475569";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(point.x, point.y, radius + 4, 0, Math.PI * 2);
    ctx.stroke();
  }
  ctx.fillStyle = "#0f172a";
  ctx.font = selected ? "bold 11px system-ui" : "11px system-ui";
  ctx.fillText(target.label || target.id, point.x + radius + 5, point.y + 4);
}

function drawPingerTarget(ctx, target, point, selected) {
  const radius = COURSE_POINT_RADIUS + (selected ? 3 : 1);
  ctx.strokeStyle = selected ? "#2563eb" : target.color_hex || "#dc2626";
  ctx.fillStyle = "#ffffff";
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
  ctx.fillStyle = "#0f172a";
  ctx.font = selected ? "bold 11px system-ui" : "11px system-ui";
  ctx.fillText(target.label || "Pinger", point.x + radius + 12, point.y + 4);
}

function drawRobotTarget(ctx, target, point, selected) {
  const radius = COURSE_POINT_RADIUS + 8 + (selected ? 3 : 0);
  ctx.fillStyle = target.color_hex || "#2563eb";
  ctx.strokeStyle = selected ? "#f97316" : "#0f172a";
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
  ctx.strokeStyle = "#1d4ed8";
  ctx.beginPath();
  ctx.arc(point.x, point.y, radius + 4, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = "#0f172a";
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
      positions[item.id] = { x: item.x, y: item.y };
    });
  }
  const robot = state.courseLayout?.robot;
  return {
    mode: state.courseLayout?.mode || "competition",
    positions,
    robot_xy: robot ? { x: robot.x, y: robot.y } : null,
  };
}

async function saveCourseLayout(reset = false) {
  const payload = courseSavePayload();
  if (!payload) {
    return;
  }
  setText("courseDialogStatus", reset ? "course layout: saving + reset" : "course layout: saving");
  const body = await postCommand({ command: "course_save", ...payload, reset });
  state.courseLayout = normalizeCourseLayout(body);
  setText("courseDialogPath", `path: ${body.path || "n/a"}`);
  setText("courseDialogStatus", body.status || "course layout: saved");
  renderCourseLayout();
  await pollStatus();
}

function bindControls() {
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
  window.addEventListener("gamepadconnected", (event) => {
    state.gamepadIndex = event.gamepad.index;
    renderGamepadStatus(`Gamepad: ${gamepadLabel(event.gamepad)} — connected`);
    pollPhysicalGamepad();
  });
  window.addEventListener("gamepaddisconnected", (event) => {
    if (event.gamepad.index === state.gamepadIndex) {
      state.gamepadIndex = null;
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
      ? "Show telemetry"
      : "Hide telemetry";
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
  $("toolsToggle").addEventListener("click", () => setToggle("toolsBody", "toolsToggle", "Show tools", "Hide tools"));
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
  $("stereoCameraApplyBtn").addEventListener("click", () => applyCameraConfig(true).catch(console.error));
  $("stereoCameraSaveBtn").addEventListener("click", () => applyCameraConfig(false).catch(console.error));
  $("stereoCameraZoomBtn").addEventListener("click", toggleStereoCameraZoom);
  document.addEventListener("keydown", (event) => {
    if (event.key === "Escape") {
      setStereoCameraExpanded(false);
    }
  });

  $("stackStartBtn").addEventListener("click", () => postCommand({ command: "stack_start" }).then(pollStatus).catch(console.error));
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
  $("missionStartBtn").addEventListener("click", () => {
    applyMissionCourseDefaults();
    setText("missionFsmStatus", "mission: starting");
    postCommand({ command: "gt_mission_start", values: missionPayload() }).then(pollStatus).catch(console.error);
  });
  $("missionCourse").addEventListener("change", applyMissionCourseDefaults);
  $("missionStopBtn").addEventListener("click", () => {
    setText("missionFsmStatus", "mission: stopping");
    postCommand({ command: "gt_mission_stop" }).then(pollStatus).catch(console.error);
  });

  $("rcReplayBrowseBtn").addEventListener("click", () => {
    $("rcReplayPath").focus();
    $("rcReplayPath").select();
  });
  $("rcReplayLoadBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_load", ...replayPayload() }).then(pollStatus).catch(console.error)
  );
  $("rcReplayPlayBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_play", ...replayPayload() }).then(pollStatus).catch(console.error)
  );
  $("rcReplayPauseBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_pause" }).then(pollStatus).catch(console.error)
  );
  $("rcReplayStopBtn").addEventListener("click", () =>
    postCommand({ command: "rc_replay_stop" }).then(pollStatus).catch(console.error)
  );
  $("rcReplaySlider").addEventListener("change", (event) =>
    postCommand({ command: "rc_replay_seek", time_s: Number(event.target.value) || 0 }).then(pollStatus).catch(console.error)
  );

  $("physicsOpenBtn").addEventListener("click", () => openPhysicsDialog().catch(console.error));
  $("courseOpenBtn").addEventListener("click", () => openCourseDialog().catch(console.error));
  $("quickPhysicsOpenBtn").addEventListener("click", () => openPhysicsDialog().catch(console.error));
  $("quickCourseOpenBtn").addEventListener("click", () => openCourseDialog().catch(console.error));
  $("physicsCloseBtn").addEventListener("click", () => $("physicsDialog").classList.add("hidden"));
  $("physicsReloadBtn").addEventListener("click", () => loadPhysicsParams().catch(console.error));
  $("physicsApplyBtn").addEventListener("click", () => applyPhysicsParams(false).catch(console.error));
  $("physicsApplyRestartBtn").addEventListener("click", () => applyPhysicsParams(true).catch(console.error));
  $("physicsRawBtn").addEventListener("click", () => openToolEditor("physics").catch(console.error));
  $("courseCloseBtn").addEventListener("click", () => $("courseDialog").classList.add("hidden"));
  $("courseReloadBtn").addEventListener("click", () => loadCourseLayout($("courseModeSelect").value).catch(console.error));
  $("courseModeSelect").addEventListener("change", (event) => loadCourseLayout(event.target.value).catch(console.error));
  $("courseUpdateSelectedBtn").addEventListener("click", updateSelectedCourseFromInputs);
  $("courseSaveBtn").addEventListener("click", () => saveCourseLayout(false).catch(console.error));
  $("courseSaveResetBtn").addEventListener("click", () => saveCourseLayout(true).catch(console.error));
  $("courseRawBtn").addEventListener("click", () => openToolEditor("course").catch(console.error));
  $("toolEditorCloseBtn").addEventListener("click", () => $("toolEditorDialog").classList.add("hidden"));
  $("toolEditorReloadBtn").addEventListener("click", () => loadToolFile(state.toolKind).catch(console.error));
  $("toolEditorSaveBtn").addEventListener("click", () => saveToolFile().catch(console.error));

  $("ping360PanelBtn").addEventListener("click", () => $("ping360Dialog").classList.remove("hidden"));
  $("ping360CloseBtn").addEventListener("click", () => $("ping360Dialog").classList.add("hidden"));
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
  $("toolEditorDialog").classList.remove("hidden");
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
setInterval(pollStatus, STATUS_POLL_MS);
setInterval(pollCameraStatus, CAMERA_POLL_MS);
setInterval(pollPhysicalGamepad, GAMEPAD_POLL_MS);
setInterval(() => {
  if (state.rcEnabled && (state.dragging || axesActive())) {
    sendRc(true);
  }
}, RC_KEEPALIVE_MS);
