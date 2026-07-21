const state = {
  websocket: null,
  reconnectTimer: null,
  ekf: {
    values: [],
    stateNames: [],
    size: 15,
  },
  bag: {
    topicsLoaded: false,
    loading: false,
    error: "",
    defaultTopics: [],
    topics: [],
    selectedTopics: [],
    analysis: {
      running: false,
      result: null,
      error: "",
    },
  },
  test: {
    running: false,
    stopping: false,
    message: "Idle",
    steps: [],
  },
  path: {
    points: [],
    pose: { x: 0, y: 0, yaw: 0 },
    options: {
      viewMode: "fit",
      color: "#6ec6ff",
      width: 2.5,
      scale: 40,
      gridStep: 1,
      showGrid: true,
      showAxes: true,
      showRobot: true,
    },
  },
  control: {
    enabled: false,
    active: false,
    sendTimer: null,
    lastErrorAt: 0,
    axes: {
      forward: 0,
      lateral: 0,
      vertical: 0,
      yaw: 0,
    },
  },
  phasePeak: {
    candidatesSignature: "",
    confirmed: false,
    selectedFrequencyHz: null,
    selectedRank: 0,
    source: "",
    selectionPending: false,
  },
  vision: {
    visible: false,
    frameTimer: null,
    frameLoading: false,
    frameSequence: 0,
    frameImage: null,
    detections: [],
    process: {},
    topics: {},
    status: {},
  },
};

const $ = (id) => document.getElementById(id);
const BAG_SELECTION_KEY = "kmu26-auv-web-gui-bag-selection";
const VISION_CONFIG_KEY = "kmu26-auv-web-gui-vision-config";

function fmt(value, digits = 2) {
  if (typeof value !== "number" || !Number.isFinite(value)) return "--";
  return value.toFixed(digits);
}

function fmtUnit(value, unit, digits = 2) {
  const text = fmt(value, digits);
  return text === "--" ? "--" : `${text} ${unit}`;
}

function fmtPercent(value, digits = 1) {
  return typeof value === "number" && Number.isFinite(value)
    ? `${(value * 100).toFixed(digits)} %`
    : "--";
}

function inputNumber(id, fallback) {
  const value = Number($(id).value);
  return Number.isFinite(value) ? value : fallback;
}

function phaseFrequenciesMatch(leftHz, rightHz) {
  if (!Number.isFinite(leftHz) || !Number.isFinite(rightHz)) return false;
  const toleranceHz = Math.max(1e-6, 1e-9 * Math.max(Math.abs(leftHz), Math.abs(rightHz)));
  return Math.abs(leftHz - rightHz) <= toleranceHz;
}

function pingerPayload(dryRun) {
  const homingMode = $("pinger-homing-mode").value || "phase";
  const referenceFrequencyHz = inputNumber("pinger-reference-frequency", 21164);
  const phaseFrequencyConfirmed = homingMode === "snr" || (
    state.phasePeak.confirmed
    && Number.isFinite(state.phasePeak.selectedFrequencyHz)
    && phaseFrequenciesMatch(state.phasePeak.selectedFrequencyHz, referenceFrequencyHz)
  );
  return {
    dry_run: dryRun,
    confirm_live: !dryRun,
    mode: "STABILIZE",
    estimator_mode: homingMode === "no_odom_phase" ? "phase" : homingMode,
    navigation_mode: homingMode === "no_odom_phase" ? "no_odom_phase" : "odometry",
    use_hydrophone_estimator: $("pinger-use-estimator").checked,
    use_audio_capture: $("pinger-use-capture").checked,
    audio_device: $("pinger-audio-device").value.trim(),
    reference_frequency_hz: referenceFrequencyHz,
    phase_peak_confirmed: phaseFrequencyConfirmed,
    phase_peak_rank: state.phasePeak.selectedRank || 0,
    probe_pwm_delta: inputNumber("pinger-probe-pwm-delta", 20),
    approach_pwm_delta: inputNumber("pinger-approach-pwm-delta", 120),
    approach_duration_s: inputNumber("pinger-approach-duration", 4),
    tank_max_depth_m: inputNumber("pinger-tank-depth", 11),
    rate_hz: inputNumber("pinger-rate", 30),
    forward_max: inputNumber("pinger-forward-max", 0.48),
    yaw_gain: inputNumber("pinger-yaw-gain", 0.85),
    yaw_command_limit: inputNumber("pinger-yaw-limit", 0.42),
    arrival_radius_m: inputNumber("pinger-arrival-radius", 1.5),
    arrival_hold_s: inputNumber("pinger-arrival-hold", 1.0),
    max_runtime_s: inputNumber("pinger-max-runtime", 180),
    success_hold_s: inputNumber("pinger-success-hold", 0.8),
    success_range_m: inputNumber("pinger-success-range", 0),
    amplitude_range_constant: inputNumber("pinger-range-constant", 0),
  };
}

function applyPingerNavigationContract({ markPreflightStale = false } = {}) {
  const homingMode = $("pinger-homing-mode").value || "phase";
  $("pinger-phase-selection").classList.toggle("disabled", homingMode === "snr");
  $("pinger-navigation-note").textContent = homingMode === "no_odom_phase"
    ? "NO_ODOM_PHASE · /odometry/filtered disabled · IMU + Bar30 depth safety"
    : homingMode === "snr"
      ? "SNR · /odometry/filtered required for the 3D gradient"
      : "PHASE · /odometry/filtered required";
  if (markPreflightStale) {
    const preflight = $("pinger-preflight-result");
    preflight.textContent = "Homing mode changed · run preflight again";
    preflight.classList.remove("good");
    preflight.classList.add("warn");
  }
  renderPingerParameterSummary();
}

function pingerParameterInputs() {
  return Array.from(document.querySelectorAll("[data-pinger-param]"));
}

function validatePingerParameters({ focusInvalid = true } = {}) {
  const successRange = $("pinger-success-range");
  const rangeConstant = $("pinger-range-constant");
  const successRangeValue = Number(successRange.value);
  const rangeConstantValue = Number(rangeConstant.value);

  rangeConstant.setCustomValidity("");
  const scanMinimum = $("pinger-scan-min-frequency");
  const scanMaximum = $("pinger-scan-max-frequency");
  const approachPwmDelta = $("pinger-approach-pwm-delta");
  const forwardMax = $("pinger-forward-max");
  scanMaximum.setCustomValidity("");
  approachPwmDelta.setCustomValidity("");
  if (successRangeValue > 0 && !(rangeConstantValue > 0)) {
    rangeConstant.setCustomValidity(
      "Success range를 사용하려면 실측한 IQ range constant를 0보다 크게 입력하십시오.",
    );
  }
  if (!(Number(scanMaximum.value) > Number(scanMinimum.value))) {
    scanMaximum.setCustomValidity("Peak scan maximum must be greater than the minimum.");
  }
  const approachPwmCap = Number(forwardMax.value) * 400;
  if (Number(approachPwmDelta.value) > approachPwmCap) {
    approachPwmDelta.setCustomValidity(
      `Approach PWM must be at most ${Math.floor(approachPwmCap)} µs for the current Forward max.`,
    );
  }

  const inputs = pingerParameterInputs();
  inputs.forEach((input) => {
    input.setAttribute("aria-invalid", String(!input.checkValidity()));
  });
  const invalid = inputs.find((input) => !input.checkValidity());
  if (invalid && focusInvalid) {
    const details = invalid.closest("details");
    if (details) details.open = true;
    invalid.focus();
    invalid.reportValidity();
  }
  return !invalid;
}

function pingerParameterNumber(id) {
  const text = $(id).value.trim();
  if (!text) return null;
  const value = Number(text);
  return Number.isFinite(value) ? value : null;
}

function renderPingerParameterSummary() {
  const homingMode = $("pinger-homing-mode").value || "phase";
  const tankDepth = pingerParameterNumber("pinger-tank-depth");
  const forwardMax = pingerParameterNumber("pinger-forward-max");
  const yawLimit = pingerParameterNumber("pinger-yaw-limit");
  const arrivalRadius = pingerParameterNumber("pinger-arrival-radius");
  const arrivalHold = pingerParameterNumber("pinger-arrival-hold");
  const maxRuntime = pingerParameterNumber("pinger-max-runtime");
  const successRange = pingerParameterNumber("pinger-success-range");
  const probePwmDelta = pingerParameterNumber("pinger-probe-pwm-delta");
  const approachPwmDelta = pingerParameterNumber("pinger-approach-pwm-delta");
  const approachDuration = pingerParameterNumber("pinger-approach-duration");
  const referenceFrequency = pingerParameterNumber("pinger-reference-frequency");
  const scanMinimum = pingerParameterNumber("pinger-scan-min-frequency");
  const scanMaximum = pingerParameterNumber("pinger-scan-max-frequency");
  const effectiveApproachPwm = approachPwmDelta === null || forwardMax === null
    ? null
    : Math.min(approachPwmDelta, forwardMax * 400);
  const chip = (label, value, className = "") =>
    `<span class="${className}">${escapeHtml(label)} <strong>${escapeHtml(value)}</strong></span>`;

  $("pinger-parameter-summary").innerHTML = [
    chip("Mode", homingMode === "no_odom_phase" ? "NO_ODOM_PHASE" : homingMode.toUpperCase()),
    chip("Tank", tankDepth === null ? "--" : `${tankDepth.toFixed(1)} m`),
    chip("Phase", referenceFrequency === null ? "--" : `${referenceFrequency.toFixed(0)} Hz`),
    chip(
      "Scan band",
      scanMinimum === null || scanMaximum === null
        ? "--"
        : `${scanMinimum.toFixed(0)}–${scanMaximum.toFixed(0)} Hz`,
    ),
    chip("Probe", probePwmDelta === null ? "--" : `1500 ± ${probePwmDelta.toFixed(0)} µs`),
    chip(
      "Approach",
      effectiveApproachPwm === null || approachDuration === null
        ? "--"
        : `1500 + ${effectiveApproachPwm.toFixed(0)} µs / ${approachDuration.toFixed(1)} s`,
    ),
    chip("Forward", forwardMax === null ? "--" : `${Math.round(forwardMax * 100)}%`),
    chip("Yaw limit", yawLimit === null ? "--" : `${Math.round(yawLimit * 100)}%`),
    chip(
      "Arrival",
      arrivalRadius === null || arrivalHold === null
        ? "--"
        : `${arrivalRadius.toFixed(1)} m / ${arrivalHold.toFixed(1)} s`,
    ),
    chip("Runtime", maxRuntime === null ? "--" : `${Math.round(maxRuntime)} s`),
    successRange > 0
      ? chip("Range stop", `${successRange.toFixed(1)} m`)
      : chip("Range stop", "OFF", "off"),
  ].join("");
}

function syncPingerParameterUi({ markPreflightStale = false } = {}) {
  validatePingerParameters({ focusInvalid: false });
  const inputs = pingerParameterInputs();
  const changed = inputs.filter((input) => input.value !== input.defaultValue);
  const invalid = inputs.filter((input) => !input.checkValidity());

  inputs.forEach((input) => {
    input.closest("label")?.classList.toggle("changed", input.value !== input.defaultValue);
  });

  const indicator = $("pinger-parameter-state");
  indicator.classList.toggle("changed", invalid.length === 0 && changed.length > 0);
  indicator.classList.toggle("invalid", invalid.length > 0);
  if (invalid.length > 0) {
    indicator.textContent = `${invalid.length} invalid value${invalid.length === 1 ? "" : "s"}`;
  } else if (changed.length > 0) {
    indicator.textContent = `${changed.length} change${changed.length === 1 ? "" : "s"} · next start`;
  } else {
    indicator.textContent = "Defaults loaded";
  }

  renderPingerParameterSummary();
  if (markPreflightStale) {
    const preflight = $("pinger-preflight-result");
    preflight.textContent = "Parameters changed · run preflight again";
    preflight.classList.remove("good");
    preflight.classList.add("warn");
  }
}

function resetPingerParameters() {
  pingerParameterInputs().forEach((input) => {
    input.value = input.defaultValue;
  });
  syncPingerParameterUi({ markPreflightStale: true });
}

function pingerRequest(path, dryRun, onSuccess = null) {
  if (!validatePingerParameters()) {
    syncPingerParameterUi({ markPreflightStale: true });
    showError(new Error("Fix invalid Pinger parameters before continuing."));
    return;
  }
  const homingMode = $("pinger-homing-mode").value || "phase";
  if (state.phasePeak.selectionPending) {
    showError(new Error("Wait for the Phase peak scanner acknowledgement."));
    return;
  }
  if (homingMode !== "snr" && !pingerPayload(dryRun).phase_peak_confirmed) {
    showError(new Error("Select a detected Phase peak or confirm the manual frequency before starting."));
    $("pinger-phase-selection").scrollIntoView({ behavior: "smooth", block: "center" });
    return;
  }
  const request = postJson(path, pingerPayload(dryRun));
  if (onSuccess) request.then(onSuccess).catch(showError);
  else request.catch(showError);
}

async function startPhasePeakScan() {
  state.phasePeak.confirmed = false;
  state.phasePeak.selectedFrequencyHz = null;
  state.phasePeak.selectedRank = 0;
  state.phasePeak.source = "";
  $("pinger-phase-selection-state").textContent = "Scanning Phase peaks · keep the vehicle still";
  $("pinger-phase-selection-state").classList.remove("good");
  $("pinger-phase-selection-state").classList.add("warn");
  await postJson("/api/pinger/phase-peaks/scan", {
    use_audio_capture: $("pinger-use-capture").checked,
    audio_device: $("pinger-audio-device").value.trim(),
    use_stamped_audio: $("pinger-use-stamped-audio").checked,
    min_frequency_hz: inputNumber("pinger-scan-min-frequency", 15000),
    max_frequency_hz: inputNumber("pinger-scan-max-frequency", 25000),
  });
}

async function choosePhaseFrequency(frequencyHz, source, rank = 0) {
  if (state.phasePeak.selectionPending) return;
  state.phasePeak.selectionPending = true;
  document.querySelectorAll('input[name="phase-peak-candidate"]').forEach((input) => {
    input.disabled = true;
  });
  try {
    const response = await postJson("/api/pinger/phase-peaks/select", {
      frequency_hz: frequencyHz,
      source,
    });
    $("pinger-reference-frequency").value = String(response.frequency_hz);
    state.phasePeak.confirmed = true;
    state.phasePeak.selectedFrequencyHz = Number(response.frequency_hz);
    state.phasePeak.selectedRank = Number(response.rank || rank || 0);
    state.phasePeak.source = response.source || source;
    syncPingerParameterUi({ markPreflightStale: true });
  } finally {
    state.phasePeak.selectionPending = false;
    state.phasePeak.candidatesSignature = "";
  }
}

async function confirmManualPhaseFrequency() {
  if (!validatePingerParameters()) return;
  const frequencyHz = inputNumber("pinger-reference-frequency", NaN);
  if (!Number.isFinite(frequencyHz)) {
    showError(new Error("Enter a valid manual reference frequency first."));
    return;
  }
  await choosePhaseFrequency(frequencyHz, "manual", 0);
}

function phasePeakMetric(value, suffix = "") {
  if (value === null || value === undefined || value === "") return "--";
  const number = Number(value);
  if (!Number.isFinite(number)) return "--";
  return `${number.toFixed(suffix === " dB" ? 1 : 3)}${suffix}`;
}

function renderPhasePeakSelection(process, selection) {
  const candidates = Array.isArray(selection.candidates) ? selection.candidates : [];
  const referenceHz = inputNumber("pinger-reference-frequency", NaN);
  const selectedHz = typeof selection.selected_frequency_hz === "number"
    ? selection.selected_frequency_hz
    : NaN;
  const selectedMatches = Boolean(
    selection.confirmed
    && Number.isFinite(selectedHz)
    && Number.isFinite(referenceHz)
    && phaseFrequenciesMatch(selectedHz, referenceHz)
  );
  state.phasePeak.confirmed = selectedMatches;
  state.phasePeak.selectedFrequencyHz = Number.isFinite(selectedHz) ? selectedHz : null;
  state.phasePeak.selectedRank = Number(selection.selected_rank || 0);
  state.phasePeak.source = selection.confirmation_source || "";

  const signature = JSON.stringify({
    candidates: candidates.map((candidate) => [
      candidate.rank,
      candidate.frequency_hz,
      candidate.magnitude,
      candidate.snr_db,
      candidate.quality,
      candidate.support,
      candidate.support_frames,
      candidate.selectable,
    ]),
    selectedHz: state.phasePeak.selectedFrequencyHz,
  });
  if (signature !== state.phasePeak.candidatesSignature) {
    state.phasePeak.candidatesSignature = signature;
    $("pinger-phase-peak-list").innerHTML = candidates.length
      ? candidates.map((candidate) => {
          const frequencyHz = Number(candidate.frequency_hz);
          const checked = Number.isFinite(selectedHz) && Math.abs(selectedHz - frequencyHz) <= 1;
          const support = Number(candidate.support);
          const supportText = Number.isFinite(support)
            ? `${(support <= 1 ? support * 100 : support).toFixed(0)}% support`
            : "support --";
          const quality = typeof candidate.quality === "string"
            ? candidate.quality
            : phasePeakMetric(candidate.quality);
          const selectable = Boolean(candidate.selectable);
          return `
            <label class="phase-peak-card ${checked ? "selected" : ""} ${selectable ? "" : "unavailable"}">
              <input type="radio" name="phase-peak-candidate" value="${frequencyHz}" data-rank="${Number(candidate.rank || 0)}" ${checked ? "checked" : ""} ${selectable ? "" : "disabled"} />
              <span class="phase-peak-rank">#${escapeHtml(candidate.rank || "-")}</span>
              <strong>${frequencyHz.toFixed(1)} Hz</strong>
              <span>SNR ${phasePeakMetric(candidate.snr_db, " dB")}</span>
              <span>mag ${phasePeakMetric(candidate.magnitude)}</span>
              <span>${escapeHtml(quality)} · ${supportText} · ${Number(candidate.support_frames || 0)} frames</span>
            </label>
          `;
        }).join("")
      : '<p class="phase-peak-empty">아직 후보가 없습니다. Scan peaks를 누르고 차량을 정지 상태로 유지하십시오.</p>';
    document.querySelectorAll('input[name="phase-peak-candidate"]').forEach((input) => {
      input.addEventListener("change", () => {
        if (!input.checked) return;
        choosePhaseFrequency(Number(input.value), "candidate", Number(input.dataset.rank || 0))
          .catch(showError);
      });
    });
  }

  const scannerRunning = Boolean(process.phase_peak_scanner_running);
  $("pinger-phase-scan").disabled = scannerRunning || Boolean(process.pinger_running)
    || state.phasePeak.selectionPending;
  $("pinger-phase-scan-stop").disabled = !scannerRunning || state.phasePeak.selectionPending;
  $("pinger-phase-manual-confirm").disabled = scannerRunning || Boolean(process.pinger_running)
    || state.phasePeak.selectionPending;
  const status = $("pinger-phase-selection-state");
  const homingMode = $("pinger-homing-mode").value || "phase";
  if (homingMode === "snr") {
    status.textContent = "SNR mode · Phase frequency confirmation is not required";
    status.classList.add("good");
    status.classList.remove("warn");
  } else if (selectedMatches) {
    status.textContent = `${selectedHz.toFixed(1)} Hz confirmed (${state.phasePeak.source || "selector"}) · homing can start`;
    status.classList.add("good");
    status.classList.remove("warn");
  } else {
    const scannerState = selection.scanner_state || "SCANNING";
    const scannerError = selection.error ? ` · ${selection.error}` : "";
    const framesSeen = Number(selection.frames_seen || 0);
    status.textContent = scannerRunning
      ? `${scannerState} · ${framesSeen} frames · ${candidates.length} candidate${
          candidates.length === 1 ? "" : "s"
        }${scannerError} · choose a stable peak`
      : candidates.length
        ? `${candidates.length} candidates ready · choose one or confirm manual Hz`
        : "Phase frequency not confirmed · homing start is blocked";
    status.classList.remove("good");
    status.classList.add("warn");
  }
}

function bindPingerParameterControls() {
  pingerParameterInputs().forEach((input) => {
    input.addEventListener("input", () => {
      syncPingerParameterUi({ markPreflightStale: true });
    });
  });
  $("pinger-parameter-reset").addEventListener("click", resetPingerParameters);
  $("pinger-homing-mode").addEventListener("change", () => {
    applyPingerNavigationContract({ markPreflightStale: true });
  });
  $("pinger-phase-scan").addEventListener("click", startPhasePeakScan);
  $("pinger-phase-scan-stop").addEventListener("click", () => {
    postJson("/api/pinger/phase-peaks/stop", {}).catch(showError);
  });
  $("pinger-phase-manual-confirm").addEventListener("click", confirmManualPhaseFrequency);
  applyPingerNavigationContract();
  syncPingerParameterUi();
}

function escapeHtml(value) {
  return String(value)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

function setPill(id, active, label) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("good", active);
  el.classList.toggle("warn", !active);
  el.classList.toggle("bad", false);
}

function setPillState(id, label, state) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("good", state === "good");
  el.classList.toggle("warn", state === "warn");
  el.classList.toggle("bad", state === "bad");
}

function setTelemetryState(id, label, state) {
  const el = $(id);
  el.textContent = label;
  el.classList.toggle("state-good", state === "good");
  el.classList.toggle("state-warn", state === "warn");
  el.classList.toggle("state-bad", state === "bad");
}

async function postJson(path, payload = {}) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!response.ok) {
    let detail = `${path} failed: ${response.status}`;
    try {
      const body = await response.json();
      if (body.detail) detail = body.detail;
    } catch (_) {
      // Keep the status-only error when the body is not JSON.
    }
    throw new Error(detail);
  }
  return response.json();
}

async function getJson(path) {
  const response = await fetch(path);
  if (!response.ok) {
    throw new Error(`${path} failed: ${response.status}`);
  }
  return response.json();
}

function bindControls() {
  bindPingerParameterControls();
  $("start-stack").addEventListener("click", () => {
    postJson("/api/stack/start").catch(showError);
  });
  $("stop-stack").addEventListener("click", () => {
    postJson("/api/stack/stop").catch(showError);
  });
  $("start-pinger-dry").addEventListener("click", () => {
    pingerRequest("/api/pinger/start", true);
  });
  $("pinger-preflight").addEventListener("click", () => {
    pingerRequest("/api/pinger/preflight", false, renderPingerPreflight);
  });
  $("pinger-set-mode").addEventListener("click", () => {
    postJson("/api/pinger/mode", { mode: $("pinger-mode").value }).catch(showError);
  });
  $("pinger-arm").addEventListener("click", () => {
    if (!window.confirm("ARM the physical vehicle? Keep the area and propellers clear.")) return;
    postJson("/api/pinger/arm", { armed: true }).catch(showError);
  });
  $("pinger-disarm").addEventListener("click", () => {
    postJson("/api/pinger/arm", { armed: false }).catch(showError);
  });
  $("start-pinger-live").addEventListener("click", () => {
    if (!validatePingerParameters()) {
      syncPingerParameterUi({ markPreflightStale: true });
      showError(new Error("Fix invalid Pinger parameters before continuing."));
      return;
    }
    if (
      !window.confirm(
        "Enable real pinger-homing RC output? The controller will command MAVROS only while the vehicle reports ARMED.",
      )
    ) {
      return;
    }
    pingerRequest("/api/pinger/start", false);
  });
  $("stop-pinger").addEventListener("click", () => {
    postJson("/api/pinger/stop").catch(showError);
  });
  $("start-bag").addEventListener("click", () => {
    startBag().catch(showError);
  });
  $("stop-bag").addEventListener("click", () => {
    postJson("/api/bag/stop").catch(showError);
  });
  $("bag-refresh-topics").addEventListener("click", () => {
    loadBagTopics().catch(showError);
  });
  document.querySelectorAll('input[name="bag-mode"]').forEach((input) => {
    input.addEventListener("change", () => {
      rememberBagTopicSelection();
      saveBagSelection().catch(showError);
    });
  });
  $("analyze-bag").addEventListener("click", () => {
    analyzeBag().catch(showError);
  });
  $("test-start").addEventListener("click", () => {
    runLocalizationTest().catch(showError);
  });
  $("test-stop").addEventListener("click", () => {
    stopLocalizationTest().catch(showError);
  });

  document.querySelectorAll("[data-dvl-command]").forEach((button) => {
    button.addEventListener("click", () => {
      const payload = {
        command: button.dataset.dvlCommand,
        parameter_name: button.dataset.dvlParam || "",
        parameter_value: button.dataset.dvlValue || "",
      };
      postJson("/api/dvl/command", payload).catch(showError);
    });
  });

  document.querySelectorAll("[data-tab]").forEach((button) => {
    button.addEventListener("click", () => {
      showTab(button.dataset.tab);
    });
  });

  $("ekf-reload").addEventListener("click", () => {
    loadEkfConfig().catch(showError);
  });
  $("ekf-save").addEventListener("click", () => {
    saveEkfConfig().catch(showError);
  });

  bindPathControls();
  bindWebControl();
  bindVisionControls();
}

function showTab(name) {
  document.querySelectorAll(".tab-button").forEach((button) => {
    button.classList.toggle("active", button.dataset.tab === name);
  });
  document.querySelectorAll(".tab-view").forEach((view) => {
    view.classList.toggle("active", view.id === `${name}-tab`);
  });
  if (name === "ekf" && state.ekf.values.length === 0) {
    loadEkfConfig().catch(showError);
  }
  if (name === "bag" && !state.bag.topicsLoaded) {
    loadBagTopics().catch(showError);
  }
  state.vision.visible = name === "vision";
  if (state.vision.visible) {
    startVisionFrameLoop();
    renderVisionCanvas();
  } else {
    stopVisionFrameLoop();
  }
}

function connectStatusSocket() {
  clearTimeout(state.reconnectTimer);
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  state.websocket = new WebSocket(`${protocol}//${window.location.host}/ws/status`);

  state.websocket.addEventListener("open", () => {
    $("connection-state").textContent = "Connected";
  });

  state.websocket.addEventListener("message", (event) => {
    renderStatus(JSON.parse(event.data));
  });

  state.websocket.addEventListener("close", () => {
    $("connection-state").textContent = "Disconnected. Reconnecting...";
    state.reconnectTimer = setTimeout(connectStatusSocket, 1000);
  });

  state.websocket.addEventListener("error", () => {
    $("connection-state").textContent = "Connection error";
  });
}

function renderStatus(payload) {
  const process = payload.process || {};
  const ros = payload.ros || {};
  const topics = ros.topics || {};
  const pose = ros.pose || {};
  const velocity = ros.velocity || {};
  const depth = ros.depth || {};
  const dvlQuality = ros.dvl_quality || {};
  const attitude = ros.attitude || {};
  const precheck = ros.precheck || {};
  const battery = ros.battery || {};
  const mavrosState = ros.mavros_state || {};
  const joy = ros.joy || {};
  const webControl = ros.web_control || {};
  const dvlConfig = ros.dvl_config || {};
  const dvlEvents = ros.dvl_events || [];
  const vision = ros.vision || {};

  setPill("stack-pill", process.stack_running, process.stack_running ? "STACK ON" : "STACK OFF");
  const pingerAlive = Boolean(process.pinger_running && topics.pinger_homing?.alive);
  const pingerStatus = ros.pinger_homing_status || {};
  const pingerMode = `${
    pingerStatus.navigation_mode === "no_odom_phase" ? "NO_ODOM " : ""
  }${pingerStatus.dry_run ? "DRY" : "LIVE"}`;
  setPill(
    "pinger-pill",
    pingerAlive,
    process.pinger_running ? `PINGER ${pingerAlive ? pingerMode : "WAIT"}` : "PINGER OFF",
  );
  renderMavrosPill(mavrosState, topics.mavros_state);
  setPill("bag-pill", process.bag_running, process.bag_running ? "BAG ON" : "BAG OFF");
  setPill("joy-pill", topics.joy?.alive, topics.joy?.alive ? "JOY ON" : "JOY OFF");
  setPill("battery-pill", topics.battery?.alive, topics.battery?.alive ? "BAT ON" : "BAT OFF");

  $("pose-value").textContent = `${fmt(pose.x)}, ${fmt(pose.y)}, ${fmt(pose.z)}`;
  $("yaw-value").textContent = `${fmt((pose.yaw || 0) * 180 / Math.PI, 1)} deg`;
  $("velocity-value").textContent = `${fmt(velocity.x)}, ${fmt(velocity.y)}, ${fmt(velocity.z)}`;
  $("depth-value").textContent = `${fmt(depth.z)} m`;
  renderDvlQuality(dvlQuality, precheck);
  renderAttitude(attitude, precheck);
  renderPrecheck(precheck);
  $("battery-voltage").textContent = fmtUnit(battery.voltage, "V");
  $("battery-current").textContent = fmtUnit(battery.current, "A");
  $("battery-soc").textContent =
    typeof battery.percentage === "number" && Number.isFinite(battery.percentage)
      ? `${fmt(battery.percentage * 100, 0)} %`
      : "--";
  $("battery-temp").textContent = fmtUnit(battery.temperature, "C", 1);
  renderMavrosState(mavrosState, topics.mavros_state);
  renderJoyGamepad(joy, topics.joy);
  renderDvl(dvlConfig, dvlEvents);
  renderTestState();
  renderBag(process);
  renderTopics(topics);
  renderPhasePeakSelection(process, ros.phase_peak_selection || {});
  renderPinger(process, ros);
  renderWebControlStatus(webControl);
  renderVision(process, topics, vision, depth);
  state.path.points = ros.path || [];
  state.path.pose = {
    x: typeof pose.x === "number" ? pose.x : 0,
    y: typeof pose.y === "number" ? pose.y : 0,
    yaw: typeof pose.yaw === "number" ? pose.yaw : 0,
  };
  renderPath();
  $("log-output").textContent = (process.logs || []).join("\n");
}

function renderDvlQuality(dvlQuality, precheck) {
  const dvlGood = precheck.dvl_good || {};
  const good = Boolean(dvlGood.ok ?? dvlQuality.good);
  const reason = dvlGood.reason || dvlQuality.reason || "--";
  const fom = typeof dvlQuality.fom === "number" ? `FOM ${fmt(dvlQuality.fom, 3)}` : "";
  const beams = typeof dvlQuality.valid_beams === "number" ? `${dvlQuality.valid_beams} beams` : "";
  const label = good ? ["GOOD", fom, beams].filter(Boolean).join(" · ") : reason;
  setTelemetryState("dvl-quality-value", label || "--", good ? "good" : "warn");
}

function renderAttitude(attitude, precheck) {
  const level = precheck.attitude_level || {};
  const tilt = typeof attitude.tilt_deg === "number" ? attitude.tilt_deg : level.tilt_deg;
  const ok = Boolean(level.ok);
  const label = typeof tilt === "number" ? `${fmt(tilt, 1)} deg` : level.reason || "--";
  setTelemetryState("attitude-tilt-value", label, ok ? "good" : "warn");
}

function renderPrecheck(precheck) {
  const ready = Boolean(precheck.ready);
  const mode = precheck.mode_ready?.mode || "--";
  const still = precheck.vehicle_still?.ok ? "still" : precheck.vehicle_still?.reason || "moving?";
  const label = ready ? `READY · ${mode}` : `BLOCKED · ${mode} · ${still}`;
  setTelemetryState("precheck-ready-value", label, ready ? "good" : "warn");
}

function bindWebControl() {
  ["forward", "lateral", "vertical", "yaw"].forEach((name) => {
    $(`control-${name}`).addEventListener("input", () => {
      updateControlAxis(name, Number($(`control-${name}`).value));
    });
  });

  $("control-enable").addEventListener("change", () => {
    setWebControlEnabled($("control-enable").checked).catch(showError);
  });

  $("control-stop").addEventListener("click", () => {
    resetControlAxes();
    state.control.active = false;
    $("control-deadman").classList.remove("active");
    sendControlCommand().catch(showThrottledControlError);
  });

  $("control-deadman").addEventListener("pointerdown", (event) => {
    event.preventDefault();
    if (!state.control.enabled) return;
    state.control.active = true;
    $("control-deadman").classList.add("active");
    sendControlCommand().catch(showThrottledControlError);
  });

  ["pointerup", "pointercancel", "pointerleave"].forEach((eventName) => {
    $("control-deadman").addEventListener(eventName, () => {
      state.control.active = false;
      $("control-deadman").classList.remove("active");
      sendControlCommand().catch(showThrottledControlError);
    });
  });

  $("control-arm").addEventListener("click", () => {
    postJson("/api/control/arm", { armed: true }).catch(showError);
  });
  $("control-disarm").addEventListener("click", () => {
    postJson("/api/control/arm", { armed: false }).catch(showError);
  });

  document.querySelectorAll("[data-control-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      postJson("/api/control/mode", { mode: button.dataset.controlMode }).catch(showError);
    });
  });

  renderControlAxisValues();
}

async function setWebControlEnabled(enabled) {
  state.control.enabled = enabled;
  state.control.active = false;
  $("control-deadman").classList.remove("active");
  if (enabled) {
    startControlLoop();
  } else {
    stopControlLoop();
  }
  await postJson("/api/control/enable", { enabled });
  await sendControlCommand();
}

function startControlLoop() {
  if (state.control.sendTimer) return;
  state.control.sendTimer = window.setInterval(() => {
    if (!state.control.enabled) return;
    sendControlCommand().catch(showThrottledControlError);
  }, 100);
}

function stopControlLoop() {
  if (!state.control.sendTimer) return;
  window.clearInterval(state.control.sendTimer);
  state.control.sendTimer = null;
}

function updateControlAxis(name, value) {
  state.control.axes[name] = clamp(value, -1, 1);
  renderControlAxisValues();
  if (state.control.enabled) {
    sendControlCommand().catch(showThrottledControlError);
  }
}

function resetControlAxes() {
  Object.keys(state.control.axes).forEach((name) => {
    state.control.axes[name] = 0;
    $(`control-${name}`).value = 0;
  });
  renderControlAxisValues();
}

function renderControlAxisValues() {
  Object.entries(state.control.axes).forEach(([name, value]) => {
    $(`control-${name}-value`).textContent = fmt(value, 2);
  });
}

async function sendControlCommand() {
  if (!state.control.enabled) return;
  await postJson("/api/control/command", {
    active: state.control.active,
    axes: state.control.axes,
  });
}

function renderWebControlStatus(webControl) {
  if (!webControl.enabled) {
    setPillState("control-state", "WEB OFF", "warn");
    $("control-publish-state").textContent = "--";
    return;
  }
  if (webControl.active && webControl.fresh) {
    setPillState("control-state", "WEB DRIVE", "bad");
  } else {
    setPillState("control-state", "WEB READY", "good");
  }
  $("control-publish-state").textContent = webControl.last_publish
    ? `Last ${webControl.last_publish}`
    : "--";
}

function showThrottledControlError(error) {
  const now = Date.now();
  if (now - state.control.lastErrorAt < 2000) return;
  state.control.lastErrorAt = now;
  showError(error);
}

function renderMavrosPill(mavrosState, topic) {
  if (!topic?.alive) {
    setPillState("mavros-pill", "MAV OFF", "warn");
    return;
  }
  if (!mavrosState.connected) {
    setPillState("mavros-pill", "MAV NO FCU", "warn");
    return;
  }
  setPillState(
    "mavros-pill",
    mavrosState.armed ? "MAV ARMED" : "MAV SAFE",
    mavrosState.armed ? "bad" : "good",
  );
}

function renderMavrosState(mavrosState, topic) {
  const alive = Boolean(topic?.alive);
  const connected = alive && Boolean(mavrosState.connected);
  const armed = connected && Boolean(mavrosState.armed);
  const mode = connected && mavrosState.mode ? mavrosState.mode : "--";
  const control = connected
    ? [
        mavrosState.guided ? "GUIDED" : "NON-GUIDED",
        mavrosState.manual_input ? "MANUAL IN" : "NO MANUAL",
      ].join(" / ")
    : "--";

  setTelemetryState(
    "mavros-connected",
    connected ? "CONNECTED" : alive ? "NO FCU" : "NO TOPIC",
    connected ? "good" : "warn",
  );
  setTelemetryState(
    "mavros-armed",
    armed ? "ARMED" : connected ? "SAFE" : "--",
    connected ? (armed ? "bad" : "good") : "warn",
  );
  setTelemetryState("mavros-mode", mode, connected ? "good" : "warn");
  setTelemetryState("mavros-control", control, connected ? "good" : "warn");
}

function renderJoyGamepad(joy, topic) {
  const axes = Array.isArray(joy.axes) ? joy.axes : [];
  const buttons = Array.isArray(joy.buttons) ? joy.buttons : [];
  const alive = Boolean(topic?.alive);
  document.querySelector(".ps4-pad")?.classList.toggle("offline", !alive);

  setJoyButton("joy-btn-cross", buttonPressed(buttons, 0));
  setJoyButton("joy-btn-circle", buttonPressed(buttons, 1));
  setJoyButton("joy-btn-square", buttonPressed(buttons, 2));
  setJoyButton("joy-btn-triangle", buttonPressed(buttons, 3));
  setJoyButton("joy-btn-l1", buttonPressed(buttons, 4));
  setJoyButton("joy-btn-r1", buttonPressed(buttons, 5));
  setJoyButton("joy-btn-share", buttonPressed(buttons, 6));
  setJoyButton("joy-btn-options", buttonPressed(buttons, 7));
  setJoyButton("joy-btn-ps", buttonPressed(buttons, 8));
  setJoyButton("joy-btn-l3", buttonPressed(buttons, 9));
  setJoyButton("joy-btn-r3", buttonPressed(buttons, 10));
  setJoyButton("joy-btn-touchpad", buttonPressed(buttons, 13));

  const dpadX = axisValue(axes, 6);
  const dpadY = axisValue(axes, 7);
  setJoyButton("joy-dpad-left", dpadX > 0.5);
  setJoyButton("joy-dpad-right", dpadX < -0.5);
  setJoyButton("joy-dpad-up", dpadY > 0.5);
  setJoyButton("joy-dpad-down", dpadY < -0.5);

  setStickDot("joy-left-stick-dot", axisValue(axes, 0), axisValue(axes, 1));
  setStickDot("joy-right-stick-dot", axisValue(axes, 2), axisValue(axes, 3));
  const l2Amount = triggerAmount(axes, 4);
  const r2Amount = triggerAmount(axes, 5);
  setTriggerFill("joy-lt-fill", l2Amount);
  setTriggerFill("joy-rt-fill", r2Amount);
  setJoyButton("joy-btn-l2", buttonPressed(buttons, 11) || l2Amount > 0.5);
  setJoyButton("joy-btn-r2", buttonPressed(buttons, 12) || r2Amount > 0.5);

  const pressedCount = buttons.filter((value) => isPressedValue(value)).length;
  const leftStick = `${fmt(axisValue(axes, 0), 2)}, ${fmt(axisValue(axes, 1), 2)}`;
  const rightStick = `${fmt(axisValue(axes, 2), 2)}, ${fmt(axisValue(axes, 3), 2)}`;
  $("joy-gamepad-state").textContent = alive
    ? `Buttons ${pressedCount} · LS ${leftStick} · RS ${rightStick}`
    : "No Joy data";
}

function setJoyButton(id, pressed) {
  const el = $(id);
  if (!el) return;
  el.classList.toggle("pressed", Boolean(pressed));
}

function setStickDot(id, x, y) {
  const el = $(id);
  if (!el) return;
  const stickSize = el.parentElement?.getBoundingClientRect().width || 72;
  const maxTravel = clamp(stickSize * 0.28, 14, 24);
  const offsetX = clamp(x, -1, 1) * maxTravel;
  const offsetY = -clamp(y, -1, 1) * maxTravel;
  el.style.transform = `translate(-50%, -50%) translate(${offsetX}px, ${offsetY}px)`;
}

function setTriggerFill(id, amount) {
  const el = $(id);
  if (!el) return;
  el.style.width = `${clamp(amount, 0, 1) * 100}%`;
}

function buttonPressed(buttons, index) {
  return isPressedValue(buttons[index]);
}

function isPressedValue(value) {
  const numeric = Number(value);
  return Number.isFinite(numeric) && numeric !== 0;
}

function axisValue(axes, index) {
  const value = Number(axes[index]);
  return Number.isFinite(value) ? value : 0;
}

function triggerAmount(axes, index) {
  if (axes[index] === undefined) return 0;
  const value = axisValue(axes, index);
  // DualShock triggers commonly report 1.0 at rest and -1.0 when fully pressed.
  if (value < -0.05) return (1 - value) / 2;
  return 0;
}

function formatCommand(command) {
  if (!command || typeof command !== "object") return "--";
  return `F ${fmt(command.forward)} | L ${fmt(command.lateral)} | H ${fmt(command.heave)} | Y ${fmt(command.yaw)}`;
}

function renderPingerPreflight(result) {
  const failed = (result.checks || []).filter((check) => !check.ok);
  $("pinger-preflight-result").textContent = result.ok
    ? "READY"
    : failed.map((check) => check.detail).join(" | ") || "NOT READY";
  $("pinger-preflight-result").classList.toggle("good", Boolean(result.ok));
  $("pinger-preflight-result").classList.toggle("warn", !result.ok);
}

function renderPinger(process, ros) {
  const pinger = ros.pinger_homing_status || {};
  const topics = ros.topics || {};
  const mux = ros.rc_mux_status || {};
  const graph = ros.graph || {};
  const depthSafety = pinger.depth_safety || {};
  const estimate = Array.isArray(pinger.estimated_source_world)
    ? pinger.estimated_source_world.map((value) => fmt(Number(value))).join(", ")
    : "--";
  const actualRange = pinger.amplitude_distance_m ?? pinger.estimated_distance_m;
  const navigationMode = pinger.navigation_mode || "odometry";
  const noOdomPhase = navigationMode === "no_odom_phase" || pinger.odometry_required === false;

  $("pinger-process-state").textContent = process.pinger_running ? "running" : "stopped";
  $("pinger-control-mode").textContent = !process.pinger_running
    ? "STOPPED"
    : pinger.dry_run
      ? `${noOdomPhase ? "NO_ODOM_PHASE · " : ""}DRY RUN · RC RELEASE`
      : pinger.control_output_active
        ? `${noOdomPhase ? "NO_ODOM_PHASE · " : ""}LIVE · RC ACTIVE`
        : `${noOdomPhase ? "NO_ODOM_PHASE · " : ""}LIVE · waiting for ARMED`;
  $("pinger-mux-state").textContent = topics.rc_mux?.alive
    ? `${mux.owner || "unknown"} | ${
        mux.conflict ? "CONFLICT" : mux.output_enabled ? "output enabled" : "output blocked"
      } | pubs ${mux.publisher_count ?? 0}`
    : `stale | /mavros/rc/override pubs ${graph.rc_output_publishers ?? 0}`;
  $("pinger-controller-state").textContent = pinger.state || "--";
  $("pinger-input-state").textContent = [
    `odom ${noOdomPhase ? "BYPASSED" : topics.odom?.alive ? "OK" : "stale"}`,
    `imu ${pinger.imu_fresh ? "OK" : "stale"}`,
    `depth ${pinger.depth_fresh ? "OK" : "stale"}`,
    `mavros ${topics.mavros_state?.alive && pinger.connected ? "OK" : "stale"}`,
    `audio ${pinger.audio_fresh ? "OK" : "stale"}`,
    `direction ${topics.hydrophone_direction?.alive ? "OK" : "stale"}`,
  ].join(" | ");
  $("pinger-estimate").textContent = `xyz ${estimate} | range ${fmtUnit(actualRange, "m")} | bearing ${fmtUnit(
    pinger.bearing_error_deg,
    "deg",
    1,
  )}`;
  $("pinger-quality").textContent = `locked ${pinger.source_locked ? "yes" : "no"} | residual ${fmtUnit(
    pinger.rms_residual_m,
    "m",
    3,
  )} | cond ${fmt(pinger.condition_number, 1)} | bias ${fmtUnit(pinger.bias_range_rate_mps, "m/s", 3)}`;
  $("pinger-requested-command").textContent = formatCommand(pinger.requested_command);
  $("pinger-command").textContent = formatCommand(pinger.command);
  $("pinger-depth-safety").textContent = `depth ${fmtUnit(depthSafety.vehicle_depth_m, "m")} / ${fmtUnit(
    depthSafety.max_vehicle_depth_m,
    "m",
  )} | probe ${fmt(depthSafety.probe_heave)} | limit ${depthSafety.limit_active ? "ON" : "off"} | recovery ${
    depthSafety.recovery_active ? "ON" : "off"
  }`;
  $("pinger-direction-source").textContent = [
    navigationMode,
    pinger.acoustic_estimator_mode || "--",
    pinger.control_direction_source || "--",
  ].join(" | ");
  const phaseSelection = ros.phase_peak_selection || {};
  const selectedFrequencyHz = typeof phaseSelection.selected_frequency_hz === "number"
    ? phaseSelection.selected_frequency_hz
    : NaN;
  $("pinger-frequency-selection").textContent = Number.isFinite(selectedFrequencyHz)
    ? `${selectedFrequencyHz.toFixed(1)} Hz | ${phaseSelection.confirmation_source || "selector"} | ${
        phaseSelection.confirmed ? "CONFIRMED" : "pending"
      }`
    : "not selected";
  $("pinger-samples").textContent = `${pinger.sample_count ?? 0} samples | probe ${pinger.probe_attempt ?? 0} / ${
    pinger.minimum_probe_legs ?? 0
  }`;
  $("pinger-result").textContent = `arrival ${pinger.arrival_complete ? "complete" : "pending"} | calibrated range ${
    pinger.range_complete ? "complete" : "pending"
  } | ${pinger.completion_reason || "running"} | runtime ${fmtUnit(pinger.active_runtime_s, "s", 1)} / ${fmtUnit(
    pinger.max_runtime_s,
    "s",
    0,
  )} | IQ K ${fmt(pinger.amplitude_range_constant, 3)}`;
  $("pinger-log-output").textContent = (process.logs || [])
    .filter(
      (line) =>
        line.includes("[pinger_homing]") ||
        line.includes("single_hydrophone_homing") ||
        line.includes("pinger_hydrophone") ||
        line.includes("phase_peak_scanner") ||
        line.includes("rc_override_mux"),
    )
    .join("\n");
}

function renderBag(process) {
  $("bag-state").textContent = process.bag_running ? "Recording" : "Stopped";
  $("bag-output").textContent = process.bag_output || "--";
  $("bag-log-output").textContent = (process.logs || [])
    .filter((line) => line.includes("[bag]") || line.includes("ros2 bag record"))
    .join("\n");
  renderBagAnalysis();
}

async function loadBagTopics() {
  state.bag.loading = true;
  state.bag.error = "";
  renderBagTopics();
  try {
    const payload = await getJson("/api/bag/topics");
    const savedSelection = loadStoredBagSelection();
    state.bag.defaultTopics = payload.default_topics || [];
    state.bag.topics = payload.topics || [];
    state.bag.selectedTopics = Array.from(
      new Set([
        ...(payload.selected_topics || []),
        ...(savedSelection.topics || []),
        ...(state.bag.selectedTopics || []),
      ]),
    );
    state.bag.topicsLoaded = true;
    setBagMode(
      typeof savedSelection.record_all === "boolean"
        ? savedSelection.record_all
        : Boolean(payload.record_all),
    );
  } catch (error) {
    state.bag.error = error.message;
    throw error;
  } finally {
    state.bag.loading = false;
    renderBagTopics();
  }
}

function renderBagTopics() {
  if (state.bag.loading) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message">Loading topics...</div>`;
    return;
  }
  if (state.bag.error) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message bad">${state.bag.error}</div>`;
    return;
  }
  if (state.bag.topics.length === 0) {
    $("bag-topic-list").innerHTML = `<div class="bag-topic-message">No topics discovered.</div>`;
    return;
  }

  const defaults = new Set(state.bag.defaultTopics);
  const selected = new Set(state.bag.selectedTopics);
  $("bag-topic-list").innerHTML = state.bag.topics
    .map((topic) => {
      const checked = defaults.has(topic) || selected.has(topic) ? "checked" : "";
      const safeTopic = escapeHtml(topic);
      return `
        <label>
          <input type="checkbox" value="${safeTopic}" ${checked} />
          <span>${safeTopic}</span>
        </label>`;
    })
    .join("");
  document.querySelectorAll("#bag-topic-list input").forEach((input) => {
    input.addEventListener("change", () => {
      rememberBagTopicSelection();
      saveBagSelection().catch(showError);
    });
  });
}

function setBagMode(recordAll) {
  const mode = recordAll ? "all" : "selected";
  const input = document.querySelector(`input[name="bag-mode"][value="${mode}"]`);
  if (input) input.checked = true;
}

function rememberBagTopicSelection() {
  state.bag.selectedTopics = Array.from(
    document.querySelectorAll("#bag-topic-list input:checked"),
  ).map((input) => input.value);
}

async function saveBagSelection() {
  const mode = document.querySelector('input[name="bag-mode"]:checked')?.value || "selected";
  const selection = {
    record_all: mode === "all",
    topics: state.bag.selectedTopics,
  };
  storeBagSelection(selection);
  await postJson("/api/bag/selection", selection);
}

function loadStoredBagSelection() {
  try {
    const saved = JSON.parse(localStorage.getItem(BAG_SELECTION_KEY) || "{}");
    return {
      record_all: typeof saved.record_all === "boolean" ? saved.record_all : undefined,
      topics: Array.isArray(saved.topics) ? saved.topics : [],
    };
  } catch (_) {
    return { topics: [] };
  }
}

function storeBagSelection(selection) {
  try {
    localStorage.setItem(BAG_SELECTION_KEY, JSON.stringify(selection));
  } catch (_) {
    // The server-side selection still updates when browser storage is unavailable.
  }
}

async function currentBagSelection({ includeDefaultTopics = false } = {}) {
  if (!state.bag.topicsLoaded) {
    await loadBagTopics();
  }
  const mode = document.querySelector('input[name="bag-mode"]:checked')?.value || "selected";
  const recordAll = mode === "all";
  rememberBagTopicSelection();
  await saveBagSelection();
  const selectedTopics = state.bag.selectedTopics;
  const topics = includeDefaultTopics
    ? Array.from(new Set([...state.bag.defaultTopics, ...selectedTopics]))
    : selectedTopics;
  if (!recordAll && topics.length === 0) {
    throw new Error("Select at least one rosbag topic.");
  }
  return {
    record_all: recordAll,
    topics,
  };
}

async function startBag() {
  const bagSelection = await currentBagSelection();
  await postJson("/api/bag/start", {
    ...bagSelection,
  });
}

async function analyzeBag() {
  state.bag.analysis.running = true;
  state.bag.analysis.error = "";
  renderBagAnalysis();
  try {
    const payload = await postJson("/api/bag/analyze", {});
    state.bag.analysis.result = payload.analysis || null;
    renderStatus(payload);
  } catch (error) {
    state.bag.analysis.error = error.message;
    throw error;
  } finally {
    state.bag.analysis.running = false;
    renderBagAnalysis();
  }
}

function renderBagAnalysis() {
  const analysis = state.bag.analysis;
  $("analyze-bag").disabled = analysis.running;
  if (analysis.running) {
    $("bag-analysis-state").textContent = "Analyzing";
    return;
  }

  const result = analysis.result;
  if (analysis.error) {
    $("bag-analysis-state").textContent = "Failed";
    $("bag-analysis-summary").innerHTML = `<div class="analysis-message bad">${escapeHtml(analysis.error)}</div>`;
    $("bag-analysis-output").textContent = analysis.error;
    drawAnalysisCanvas(null);
    return;
  }
  if (!result) {
    $("bag-analysis-state").textContent = "--";
    $("bag-analysis-summary").innerHTML = "";
    $("bag-analysis-output").textContent = "--";
    drawAnalysisCanvas(null);
    return;
  }

  const status = result.assessment?.status || "--";
  $("bag-analysis-state").textContent = status.toUpperCase();
  $("bag-analysis-summary").innerHTML = analysisSummaryHtml(result);
  $("bag-analysis-output").textContent = analysisText(result);
  drawAnalysisCanvas(result);
}

function analysisSummaryHtml(result) {
  const filtered = result.odometry?.filtered || {};
  const dvlOdom = result.odometry?.dvl_odom || {};
  const dvlDr = result.odometry?.dvl_dr || {};
  const dvlData = result.dvl?.data || {};
  const dvlTwist = result.dvl?.twist || {};
  return [
    analysisMetric("Filtered Length", fmtUnit(filtered.path_length_m, "m")),
    analysisMetric("Closure", fmtUnit(filtered.start_end_error_m, "m")),
    analysisMetric("Est. Laps", fmt(Math.abs(filtered.estimated_laps || 0), 2)),
    analysisMetric("DVL Valid", fmtPercent(dvlData.valid_rate)),
    analysisMetric("DVL FOM", fmt(dvlData.fom?.mean, 3)),
    analysisMetric("DVL Alt Min", fmtUnit(dvlData.altitude_m?.min, "m", 2)),
    analysisMetric("DVL Odom Length", fmtUnit(dvlOdom.path_length_m, "m")),
    analysisMetric("DVL DR Length", fmtUnit(dvlDr.path_length_m, "m")),
    analysisMetric("Twist Speed Max", fmtUnit(dvlTwist.speed_mps?.max, "m/s", 2)),
  ].join("");
}

function analysisMetric(label, value) {
  return `
    <div>
      <span>${escapeHtml(label)}</span>
      <strong>${escapeHtml(value || "--")}</strong>
    </div>`;
}

function analysisText(result) {
  const lines = [
    `Bag: ${result.bag_path || "--"}`,
    `Report Dir: ${result.report_dir || "--"}`,
    `Result File: ${result.result_path || "--"}`,
    `Image File: ${result.image_path || "--"}`,
    `Generated: ${result.generated_at || "--"}`,
    "",
    "Notes:",
  ];
  const notes = result.assessment?.notes || [];
  if (notes.length === 0) {
    lines.push("OK");
  } else {
    notes.forEach((note) => {
      lines.push(`${String(note.level || "info").toUpperCase()} ${note.message || ""}`);
    });
  }
  return lines.join("\n");
}

function drawAnalysisCanvas(result) {
  const canvas = $("analysis-canvas");
  const { ctx, width, height } = resizePathCanvas(canvas);
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b0f0e";
  ctx.fillRect(0, 0, width, height);

  if (!result) return;

  const tracks = [
    { name: "filtered", label: "EKF", color: "#6ec6ff", points: result.samples?.filtered || [] },
    { name: "dvl_odom", label: "DVL Odom", color: "#52d273", points: result.samples?.dvl_odom || [] },
    { name: "dvl_dr", label: "DVL DR", color: "#f0b84f", points: result.samples?.dvl_dr || [] },
  ].filter((track) => track.points.length > 1);
  if (tracks.length === 0) return;

  const allPoints = tracks.flatMap((track) => track.points);
  const xs = allPoints.map((point) => point.x);
  const ys = allPoints.map((point) => point.y);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 1);
  const spanY = Math.max(maxY - minY, 1);
  const view = {
    centerX: (minX + maxX) / 2,
    centerY: (minY + maxY) / 2,
    pixelsPerMeter: clamp(0.78 * Math.min(width / spanX, height / spanY), 2, 240),
  };

  drawPathGrid(ctx, view, width, height, { gridStep: niceGridStep(Math.max(spanX, spanY)) });
  drawPathAxes(ctx, view, width, height);
  tracks.forEach((track) => drawAnalysisTrack(ctx, track, view, width, height));
  drawAnalysisLegend(ctx, tracks);
}

function drawAnalysisTrack(ctx, track, view, width, height) {
  ctx.save();
  ctx.lineWidth = track.name === "filtered" ? 2.5 : 1.8;
  ctx.strokeStyle = track.color;
  ctx.globalAlpha = track.name === "filtered" ? 1 : 0.78;
  ctx.beginPath();
  track.points.forEach((point, index) => {
    const screen = pathToScreen(point, view, width, height);
    if (index === 0) ctx.moveTo(screen.x, screen.y);
    else ctx.lineTo(screen.x, screen.y);
  });
  ctx.stroke();

  const first = pathToScreen(track.points[0], view, width, height);
  const last = pathToScreen(track.points[track.points.length - 1], view, width, height);
  ctx.globalAlpha = 1;
  ctx.fillStyle = track.color;
  ctx.beginPath();
  ctx.arc(first.x, first.y, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = track.color;
  ctx.strokeRect(last.x - 4, last.y - 4, 8, 8);
  ctx.restore();
}

function drawAnalysisLegend(ctx, tracks) {
  ctx.save();
  ctx.font = "12px sans-serif";
  ctx.textBaseline = "top";
  let x = 12;
  const y = 12;
  tracks.forEach((track) => {
    ctx.fillStyle = track.color;
    ctx.fillRect(x, y + 4, 16, 3);
    ctx.fillStyle = "#dce8e1";
    ctx.fillText(track.label, x + 22, y);
    x += ctx.measureText(track.label).width + 54;
  });
  ctx.restore();
}

function niceGridStep(span) {
  if (span <= 2) return 0.25;
  if (span <= 5) return 0.5;
  if (span <= 12) return 1;
  if (span <= 30) return 2;
  return 5;
}

async function runLocalizationTest() {
  state.test.running = true;
  state.test.message = "Running";
  state.test.steps = [];
  renderTestState();
  try {
    const payload = await postJson("/api/test/start", {});
    const test = payload.test || {};
    state.test.message = test.message || "Ready";
    state.test.steps = test.steps || [];
    renderStatus(payload);
  } catch (error) {
    state.test.message = "Failed";
    throw error;
  } finally {
    state.test.running = false;
    renderTestState();
  }
}

async function stopLocalizationTest() {
  state.test.stopping = true;
  state.test.message = "Stopping";
  renderTestState();
  try {
    const payload = await postJson("/api/test/stop", {});
    const test = payload.test || {};
    state.test.message = test.message || "Idle";
    state.test.steps = test.steps || [];
    renderStatus(payload);
  } catch (error) {
    state.test.message = "Stop failed";
    throw error;
  } finally {
    state.test.running = false;
    state.test.stopping = false;
    renderTestState();
  }
}

function renderTestState() {
  $("test-state").textContent = state.test.running ? "Running" : state.test.message;
  $("test-start").disabled = state.test.running || state.test.stopping;
  $("test-stop").disabled = state.test.stopping;
}

function renderDvl(config, events) {
  const hasConfig = Object.keys(config).length > 0;
  $("dvl-updated").textContent = config.updated_at || "--";
  $("dvl-range").textContent = config.range_mode || "--";
  $("dvl-acoustic").textContent =
    typeof config.acoustic_enabled === "boolean" ? (config.acoustic_enabled ? "ON" : "OFF") : "--";
  $("dvl-dark").textContent =
    typeof config.dark_mode_enabled === "boolean" ? (config.dark_mode_enabled ? "ON" : "OFF") : "--";
  $("dvl-sound").textContent =
    typeof config.speed_of_sound === "number" ? `${config.speed_of_sound} m/s` : "--";
  $("dvl-rotation").textContent =
    typeof config.mounting_rotation_offset === "number" ? `${config.mounting_rotation_offset} deg` : "--";
  $("dvl-error").textContent = config.error_message || "--";
  $("dvl-config-json").textContent = hasConfig ? JSON.stringify(dvlConfigPayload(config), null, 2) : "--";

  const last = events[events.length - 1];
  if (last) {
    const name = last.parameter_name ? `${last.command}.${last.parameter_name}` : last.command;
    $("dvl-last").textContent = `${last.success ? "OK" : "FAIL"} ${name}`;
  } else {
    $("dvl-last").textContent = "--";
  }

  $("dvl-events").textContent = events
    .slice(-12)
    .map((event) => {
      const name = event.parameter_name ? `${event.command}.${event.parameter_name}` : event.command;
      const value = event.parameter_value ? ` ${event.parameter_value}` : "";
      const result = event.success ? "OK" : `FAIL ${event.error_message || ""}`.trim();
      const label = event.type === "config" ? "config received" : event.type;
      return `${event.time} ${label} ${name}${value} ${result}`;
    })
    .join("\n");
}

async function loadEkfConfig() {
  $("ekf-save-state").textContent = "Loading";
  const payload = await getJson("/api/ekf/process_noise");
  state.ekf.values = payload.values || [];
  state.ekf.stateNames = payload.state_names || [];
  state.ekf.size = payload.size || 15;
  $("ekf-path").textContent = payload.path || "--";
  $("ekf-save-state").textContent = "Loaded";
  renderEkfEditor();
}

async function saveEkfConfig() {
  const values = readEkfMatrix();
  $("ekf-save-state").textContent = "Saving";
  const payload = await postJson("/api/ekf/process_noise", { values });
  state.ekf.values = payload.values || values;
  $("ekf-save-state").textContent = "Saved";
  renderEkfEditor();
}

function renderEkfEditor() {
  const { values, stateNames, size } = state.ekf;
  if (values.length !== size * size) return;

  $("ekf-diagonal").innerHTML = stateNames
    .map((name, index) => {
      const value = values[index * size + index];
      return `
        <label>
          <span>${name}</span>
          <input type="number" step="0.001" data-ekf-row="${index}" data-ekf-col="${index}" value="${value}" />
        </label>`;
    })
    .join("");

  const header = [
    '<div class="ekf-cell ekf-corner"></div>',
    ...stateNames.map((name) => `<div class="ekf-cell ekf-heading">${name}</div>`),
  ].join("");

  const rows = [];
  for (let row = 0; row < size; row += 1) {
    rows.push(`<div class="ekf-cell ekf-heading">${stateNames[row]}</div>`);
    for (let col = 0; col < size; col += 1) {
      const value = values[row * size + col];
      rows.push(
        `<input class="ekf-cell ekf-input" type="number" step="0.001" data-ekf-row="${row}" data-ekf-col="${col}" value="${value}" />`,
      );
    }
  }
  $("ekf-matrix").innerHTML = header + rows.join("");

  document.querySelectorAll("[data-ekf-row]").forEach((input) => {
    input.addEventListener("change", syncEkfInputs);
  });
}

function syncEkfInputs(event) {
  const row = Number(event.target.dataset.ekfRow);
  const col = Number(event.target.dataset.ekfCol);
  const size = state.ekf.size;
  const value = Number(event.target.value);
  state.ekf.values[row * size + col] = Number.isFinite(value) ? value : 0;
  document.querySelectorAll(`[data-ekf-row="${row}"][data-ekf-col="${col}"]`).forEach((input) => {
    if (input !== event.target) input.value = event.target.value;
  });
}

function readEkfMatrix() {
  const { size } = state.ekf;
  const values = [...state.ekf.values];
  document.querySelectorAll("#ekf-matrix .ekf-input").forEach((input) => {
    const row = Number(input.dataset.ekfRow);
    const col = Number(input.dataset.ekfCol);
    const value = Number(input.value);
    values[row * size + col] = Number.isFinite(value) ? value : 0;
  });
  return values;
}

function dvlConfigPayload(config) {
  return {
    range_mode: config.range_mode,
    acoustic_enabled: config.acoustic_enabled,
    dark_mode_enabled: config.dark_mode_enabled,
    speed_of_sound: config.speed_of_sound,
    mounting_rotation_offset: config.mounting_rotation_offset,
    response_to: config.response_to,
    success: config.success,
    error_message: config.error_message,
    format: config.format,
    type: config.type,
    updated_at: config.updated_at,
  };
}

function renderTopics(topics) {
  const rows = Object.entries(topics)
    .filter(([name]) => !name.startsWith("vision_"))
    .map(([, topic]) => {
    const age = typeof topic.age === "number" ? `${fmt(topic.age, 2)}s` : "--";
    const hz = `${fmt(topic.hz, 1)} Hz`;
    const cls = topic.alive ? "alive" : "stale";
    return `
      <div class="topic ${cls}">
        <strong>${topic.name}</strong>
        <span>${topic.alive ? "alive" : "stale"} · ${hz} · age ${age}</span>
      </div>`;
    });
  $("topic-list").innerHTML = rows.join("");
}

const PATH_OPTIONS_KEY = "kmu26-auv-web-gui-path-options";

function bindPathControls() {
  loadPathOptions();
  syncPathControls();

  [
    "path-view-mode",
    "path-color",
    "path-width",
    "path-scale",
    "path-grid-step",
    "path-show-grid",
    "path-show-axes",
    "path-show-robot",
  ].forEach((id) => {
    $(id).addEventListener("input", updatePathOptionsFromControls);
    $(id).addEventListener("change", updatePathOptionsFromControls);
  });

  $("path-clear").addEventListener("click", () => {
    state.path.points = [];
    renderPath();
    postJson("/api/path/clear").catch(showError);
  });
  $("path-set-origin").addEventListener("click", () => {
    postJson("/api/localization/set_origin").then(renderStatus).catch(showError);
  });

  window.addEventListener("resize", renderPath);
}

function loadPathOptions() {
  try {
    const saved = JSON.parse(localStorage.getItem(PATH_OPTIONS_KEY) || "{}");
    state.path.options = { ...state.path.options, ...saved };
  } catch (_) {
    // Keep defaults when saved options are not parseable.
  }
}

function savePathOptions() {
  try {
    localStorage.setItem(PATH_OPTIONS_KEY, JSON.stringify(state.path.options));
  } catch (_) {
    // The view still updates when browser storage is unavailable.
  }
}

function syncPathControls() {
  const options = state.path.options;
  $("path-view-mode").value = options.viewMode;
  $("path-color").value = options.color;
  $("path-width").value = options.width;
  $("path-scale").value = options.scale;
  $("path-grid-step").value = options.gridStep;
  $("path-show-grid").checked = options.showGrid;
  $("path-show-axes").checked = options.showAxes;
  $("path-show-robot").checked = options.showRobot;
  renderPathOptionValues();
}

function updatePathOptionsFromControls() {
  state.path.options = {
    viewMode: $("path-view-mode").value,
    color: $("path-color").value,
    width: clamp(Number($("path-width").value), 1, 12),
    scale: clamp(Number($("path-scale").value), 5, 200),
    gridStep: clamp(Number($("path-grid-step").value), 0.1, 20),
    showGrid: $("path-show-grid").checked,
    showAxes: $("path-show-axes").checked,
    showRobot: $("path-show-robot").checked,
  };
  renderPathOptionValues();
  savePathOptions();
  renderPath();
}

function renderPathOptionValues() {
  $("path-width-value").textContent = `${fmt(state.path.options.width, 1)} px`;
  $("path-scale-value").textContent = `${fmt(state.path.options.scale, 0)} px/m`;
}

function clamp(value, min, max) {
  if (!Number.isFinite(value)) return min;
  return Math.min(max, Math.max(min, value));
}

function resizePathCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function renderPath() {
  const points = state.path.points;
  const options = state.path.options;
  const canvas = $("path-canvas");
  const { ctx, width, height } = resizePathCanvas(canvas);
  const view = computePathView(points, state.path.pose, options, width, height);

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#0b0f0e";
  ctx.fillRect(0, 0, width, height);

  if (options.showGrid) drawPathGrid(ctx, view, width, height, options);
  if (options.showAxes) drawPathAxes(ctx, view, width, height);
  drawPathTrail(ctx, points, view, options);
  if (options.showRobot) drawRobotMarker(ctx, state.path.pose, view);

  $("path-point-count").textContent = `${points.length} points`;
  $("path-view-state").textContent = `${view.label} · ${fmt(view.pixelsPerMeter, 1)} px/m`;
}

function computePathView(points, pose, options, width, height) {
  if (options.viewMode === "fit" && points.length > 1) {
    const xs = points.map((p) => p.x);
    const ys = points.map((p) => p.y);
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minY = Math.min(...ys);
    const maxY = Math.max(...ys);
    const spanX = Math.max(maxX - minX, options.gridStep);
    const spanY = Math.max(maxY - minY, options.gridStep);
    return {
      centerX: (minX + maxX) / 2,
      centerY: (minY + maxY) / 2,
      pixelsPerMeter: clamp(0.82 * Math.min(width / spanX, height / spanY), 5, 200),
      label: "Fit",
    };
  }

  if (options.viewMode === "follow" && points.length > 0) {
    const last = points[points.length - 1];
    return {
      centerX: last.x,
      centerY: last.y,
      pixelsPerMeter: options.scale,
      label: "Follow",
    };
  }

  return {
    centerX: 0,
    centerY: 0,
    pixelsPerMeter: options.scale,
    label: "Map",
  };
}

function pathToScreen(point, view, width, height) {
  return {
    x: width / 2 + (point.x - view.centerX) * view.pixelsPerMeter,
    y: height / 2 - (point.y - view.centerY) * view.pixelsPerMeter,
  };
}

function drawPathGrid(ctx, view, width, height, options) {
  const step = options.gridStep;
  const left = view.centerX - width / 2 / view.pixelsPerMeter;
  const right = view.centerX + width / 2 / view.pixelsPerMeter;
  const bottom = view.centerY - height / 2 / view.pixelsPerMeter;
  const top = view.centerY + height / 2 / view.pixelsPerMeter;
  const startX = Math.floor(left / step) * step;
  const startY = Math.floor(bottom / step) * step;

  ctx.lineWidth = 1;
  for (let x = startX; x <= right; x += step) {
    const screen = pathToScreen({ x, y: 0 }, view, width, height);
    const major = Math.abs(Math.round(x / step)) % 5 === 0;
    ctx.strokeStyle = major ? "#30413a" : "#22302b";
    ctx.beginPath();
    ctx.moveTo(screen.x, 0);
    ctx.lineTo(screen.x, height);
    ctx.stroke();
  }
  for (let y = startY; y <= top; y += step) {
    const screen = pathToScreen({ x: 0, y }, view, width, height);
    const major = Math.abs(Math.round(y / step)) % 5 === 0;
    ctx.strokeStyle = major ? "#30413a" : "#22302b";
    ctx.beginPath();
    ctx.moveTo(0, screen.y);
    ctx.lineTo(width, screen.y);
    ctx.stroke();
  }
}

function drawPathAxes(ctx, view, width, height) {
  const origin = pathToScreen({ x: 0, y: 0 }, view, width, height);
  ctx.lineWidth = 1.5;
  if (origin.y >= 0 && origin.y <= height) {
    ctx.strokeStyle = "#9d3343";
    ctx.beginPath();
    ctx.moveTo(0, origin.y);
    ctx.lineTo(width, origin.y);
    ctx.stroke();
  }
  if (origin.x >= 0 && origin.x <= width) {
    ctx.strokeStyle = "#2a8e65";
    ctx.beginPath();
    ctx.moveTo(origin.x, 0);
    ctx.lineTo(origin.x, height);
    ctx.stroke();
  }
}

function drawPathTrail(ctx, points, view, options) {
  if (points.length < 2) return;
  const canvas = $("path-canvas");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const stride = Math.max(1, Math.ceil(points.length / 20000));

  ctx.strokeStyle = options.color;
  ctx.lineWidth = options.width;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.beginPath();
  let started = false;
  points.forEach((point, index) => {
    if (index % stride !== 0 && index !== points.length - 1) return;
    if (!Number.isFinite(point.x) || !Number.isFinite(point.y)) return;
    const screen = pathToScreen(point, view, width, height);
    if (!started) {
      ctx.moveTo(screen.x, screen.y);
      started = true;
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.stroke();
}

function drawRobotMarker(ctx, pose, view) {
  const canvas = $("path-canvas");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const center = pathToScreen(pose, view, width, height);
  const yaw = Number.isFinite(pose.yaw) ? pose.yaw : 0;
  const size = 9;

  ctx.save();
  ctx.translate(center.x, center.y);
  ctx.rotate(-yaw);
  ctx.fillStyle = "#52d273";
  ctx.strokeStyle = "#d8ffe3";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(size + 3, 0);
  ctx.lineTo(-size, -size * 0.65);
  ctx.lineTo(-size * 0.55, 0);
  ctx.lineTo(-size, size * 0.65);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function bindVisionControls() {
  loadVisionConfig();
  document.querySelectorAll("[data-vision-yolo], [data-vision-mission]").forEach((input) => {
    input.addEventListener("change", saveVisionConfig);
  });

  $("vision-start-yolo").addEventListener("click", () => {
    startVisionYolo().catch(showError);
  });
  $("vision-stop-yolo").addEventListener("click", () => {
    postJson("/api/vision/yolo/stop").catch(showError);
  });
  $("vision-start-mission").addEventListener("click", () => {
    startVisionMission().catch(showError);
  });
  $("vision-stop-mission").addEventListener("click", () => {
    postJson("/api/vision/mission/stop").catch(showError);
  });
  $("vision-start-all").addEventListener("click", () => {
    startVisionStack().catch(showError);
  });
  $("vision-stop-all").addEventListener("click", () => {
    postJson("/api/vision/stop").catch(showError);
  });
  $("vision-enable-control").addEventListener("click", () => {
    toggleVisionControl().catch(showError);
  });
  $("vision-emergency-stop").addEventListener("click", () => {
    postJson("/api/vision/emergency_stop").catch(showError);
  });
  window.addEventListener("resize", renderVisionCanvas);
  renderVisionRcChannels([]);
}

function visionLaunchArgs(attribute) {
  const args = {};
  document.querySelectorAll(`[${attribute}]`).forEach((input) => {
    const key = input.getAttribute(attribute);
    if (!key) return;
    args[key] = input.type === "checkbox" ? String(input.checked) : input.value.trim();
  });
  return args;
}

async function startVisionYolo() {
  saveVisionConfig();
  return postJson("/api/vision/yolo/start", {
    launch_args: visionLaunchArgs("data-vision-yolo"),
  });
}

async function startVisionMission() {
  saveVisionConfig();
  return postJson("/api/vision/mission/start", {
    launch_args: visionLaunchArgs("data-vision-mission"),
  });
}

async function startVisionStack() {
  saveVisionConfig();
  if (!state.vision.process.vision_yolo_running) {
    await startVisionYolo();
  }
  if (!state.vision.process.vision_mission_running) {
    await startVisionMission();
  }
}

async function toggleVisionControl() {
  const enabled = Boolean(state.vision.status.mission_enabled);
  if (!enabled) {
    const accepted = window.confirm(
      "Enable autonomous mission control? Verify camera, depth sign, MAVROS mode, and PWM direction first.",
    );
    if (!accepted) return;
  }
  await postJson("/api/vision/control", { enabled: !enabled });
}

function saveVisionConfig() {
  const values = {};
  document.querySelectorAll("[data-vision-yolo], [data-vision-mission]").forEach((input) => {
    const group = input.hasAttribute("data-vision-yolo") ? "yolo" : "mission";
    const name = input.getAttribute(`data-vision-${group}`);
    values[`${group}.${name}`] = input.type === "checkbox" ? input.checked : input.value;
  });
  localStorage.setItem(VISION_CONFIG_KEY, JSON.stringify(values));
}

function loadVisionConfig() {
  let values = {};
  try {
    values = JSON.parse(localStorage.getItem(VISION_CONFIG_KEY) || "{}");
  } catch (_) {
    values = {};
  }
  Object.entries(values).forEach(([key, value]) => {
    const [group, name] = key.split(".", 2);
    const input = document.querySelector(`[data-vision-${group}="${name}"]`);
    if (!input) return;
    if (input.type === "checkbox") input.checked = Boolean(value);
    else input.value = value;
  });
}

function startVisionFrameLoop() {
  if (state.vision.frameTimer) return;
  refreshVisionFrame().catch(showThrottledVisionError);
  state.vision.frameTimer = window.setInterval(() => {
    refreshVisionFrame().catch(showThrottledVisionError);
  }, 100);
}

function stopVisionFrameLoop() {
  if (!state.vision.frameTimer) return;
  window.clearInterval(state.vision.frameTimer);
  state.vision.frameTimer = null;
}

async function refreshVisionFrame() {
  if (state.vision.frameLoading || !state.vision.visible) return;
  state.vision.frameLoading = true;
  try {
    const response = await fetch(`/api/vision/frame?after=${state.vision.frameSequence}`, {
      cache: "no-store",
    });
    if (response.status === 204) return;
    if (!response.ok) throw new Error(`vision frame failed: ${response.status}`);
    const sequence = Number(response.headers.get("X-Vision-Frame-Sequence") || 0);
    const blob = await response.blob();
    const image = await decodeVisionImage(blob);
    if (state.vision.frameImage?.close) state.vision.frameImage.close();
    state.vision.frameImage = image;
    state.vision.frameSequence = sequence;
    $("vision-feed-empty").classList.add("hidden");
    renderVisionCanvas();
  } finally {
    state.vision.frameLoading = false;
  }
}

async function decodeVisionImage(blob) {
  if (window.createImageBitmap) return window.createImageBitmap(blob);
  const url = URL.createObjectURL(blob);
  try {
    const image = new Image();
    image.src = url;
    await image.decode();
    return image;
  } finally {
    URL.revokeObjectURL(url);
  }
}

function renderVision(process, topics, vision, depth) {
  state.vision.process = process;
  state.vision.topics = topics;
  state.vision.status = vision;
  state.vision.detections = vision.detections || [];

  const annotatedFeedAlive = Boolean(topics.vision_camera?.alive);
  const bboxAlive = Boolean(topics.vision_bbox?.alive);
  const yoloRunning = Boolean(process.vision_yolo_running);
  const missionRunning = Boolean(process.vision_mission_running);
  const missionAlive = Boolean(topics.vision_mission_state?.alive)
    || (vision.mission_state && vision.mission_state !== "UNKNOWN" && topics.vision_rc_command?.alive);
  const controlEnabled = Boolean(vision.mission_enabled);

  setPillState(
    "vision-camera-pill",
    annotatedFeedAlive ? "YOLO FEED LIVE" : yoloRunning ? "YOLO FEED WAIT" : "YOLO FEED OFF",
    annotatedFeedAlive ? "good" : "warn",
  );
  setPillState(
    "vision-yolo-pill",
    bboxAlive ? (yoloRunning ? "YOLO LOCAL" : "YOLO DDS") : yoloRunning ? "YOLO WAIT" : "YOLO OFF",
    bboxAlive ? "good" : yoloRunning ? "warn" : "bad",
  );
  setPillState(
    "vision-mission-pill",
    missionAlive ? (missionRunning ? "MISSION LOCAL" : "MISSION DDS") : missionRunning ? "MISSION WAIT" : "MISSION OFF",
    missionAlive ? "good" : missionRunning ? "warn" : "bad",
  );
  setPillState(
    "vision-control-pill",
    controlEnabled ? "AUTO CONTROL ON" : "CONTROL OFF",
    controlEnabled ? "bad" : "warn",
  );

  const missionState = vision.mission_state || "UNKNOWN";
  $("vision-state-value").textContent = missionState;
  $("vision-state-age").textContent = topicAgeText(topics.vision_mission_state);
  document.querySelectorAll("[data-vision-state]").forEach((item) => {
    const active = item.dataset.visionState === missionState;
    item.classList.toggle("active", active);
    item.classList.toggle("failsafe", active && missionState === "FAILSAFE");
  });

  const depthScale = Number(document.querySelector('[data-vision-mission="depth_pose_scale"]').value);
  const depthOffset = Number(document.querySelector('[data-vision-mission="depth_pose_offset_m"]').value);
  const missionDepth = typeof depth.z === "number"
    ? depth.z * depthScale + depthOffset
    : null;
  $("vision-depth-value").textContent = fmtUnit(missionDepth, "m");
  renderVisionDetections(state.vision.detections, bboxAlive);
  renderVisionRcChannels(vision.rc_channels || []);
  renderVisionCanvas();

  $("vision-start-yolo").disabled = yoloRunning;
  $("vision-stop-yolo").disabled = !yoloRunning;
  $("vision-start-mission").disabled = missionRunning;
  $("vision-stop-mission").disabled = !missionRunning;
  $("vision-start-all").disabled = yoloRunning && missionRunning;
  $("vision-stop-all").disabled = !yoloRunning && !missionRunning;
  $("vision-enable-control").textContent = controlEnabled ? "Disable Auto Mission" : "Enable Auto Mission";
  $("vision-enable-control").classList.toggle("danger", controlEnabled);

  const logs = (process.logs || []).filter(
    (line) => line.includes("[vision_") || line.includes("auv_buoy_vision_control"),
  );
  $("vision-log-output").textContent = logs.slice(-80).join("\n") || "Vision process logs will appear here.";

  const frameSize = vision.frame_width && vision.frame_height
    ? `${vision.frame_width}x${vision.frame_height}`
    : "--";
  $("vision-frame-meta").textContent = annotatedFeedAlive
    ? `${frameSize} · frame ${vision.frame_sequence || 0} · ${fmt(topics.vision_camera?.hz, 1)} Hz`
    : "Waiting for completed YOLO frame";
}

function topicAgeText(topic) {
  if (!topic || typeof topic.age !== "number") return "No topic";
  return `${fmt(topic.age, 2)} s ago · ${fmt(topic.hz, 1)} Hz`;
}

function renderVisionDetections(detections, bboxAlive) {
  const fresh = detections.filter((item) => typeof item.age !== "number" || item.age <= 1.5);
  const buoyClass = Number($("vision-buoy-class").value);
  const stickClass = Number($("vision-stick-class").value);
  if (!fresh.length) {
    $("vision-detections").innerHTML = `<div class="vision-detection-empty">${bboxAlive ? "YOLO active · no target" : "No YOLO bbox topic"}</div>`;
    $("vision-detection-value").textContent = bboxAlive ? "NO TARGET" : "OFFLINE";
    $("vision-error-value").textContent = "--";
    $("vision-area-value").textContent = "--";
    return;
  }

  const classLabel = (classId) => {
    if (classId === buoyClass) return "BUOY";
    if (classId === stickClass) return "STICK";
    return `CLASS ${classId}`;
  };
  $("vision-detections").innerHTML = fresh.map((item) => `
    <div class="vision-detection-card">
      <strong>${classLabel(item.class_id)}</strong><strong>${fmtPercent(item.confidence, 1)}</strong>
      <span>error x/y</span><span>${fmt(item.error_x, 3)} / ${fmt(item.error_y, 3)}</span>
      <span>area</span><span>${fmtPercent(item.area_ratio, 2)}</span>
    </div>
  `).join("");
  const target = fresh[0];
  $("vision-detection-value").textContent = `${classLabel(target.class_id)} ${fmtPercent(target.confidence, 1)}`;
  $("vision-error-value").textContent = `${fmt(target.error_x, 3)}, ${fmt(target.error_y, 3)}`;
  $("vision-area-value").textContent = fmtPercent(target.area_ratio, 2);
}

function renderVisionRcChannels(channels) {
  const configuredControls = [
    { parameter: "throttle_channel", fallback: 3, label: "Vertical" },
    { parameter: "yaw_channel", fallback: 4, label: "Yaw" },
    { parameter: "forward_channel", fallback: 5, label: "Forward" },
  ];
  const labelsByChannel = new Map(configuredControls.map((item) => {
    const input = document.querySelector(`[data-vision-mission="${item.parameter}"]`);
    const configuredChannel = Number(input?.value);
    const channel = Number.isInteger(configuredChannel) && configuredChannel >= 1 && configuredChannel <= 18
      ? configuredChannel
      : item.fallback;
    return [channel, item.label];
  }));
  $("vision-rc-grid").innerHTML = Array.from({ length: 18 }, (_, index) => {
    const channel = index + 1;
    const label = labelsByChannel.get(channel) || "RC Channel";
    const receivedValue = channels[channel - 1];
    const value = typeof receivedValue === "number" ? Math.round(receivedValue) : null;
    const isPwm = value !== null && value !== 0 && value !== 65535;
    let displayValue = value === null ? "--" : String(value);
    let detail = "NO MESSAGE";
    if (value === 0) {
      displayValue = "RELEASE";
      detail = "RAW 0";
    } else if (value === 65535) {
      displayValue = "NO COMMAND";
      detail = "RAW 65535";
    } else if (isPwm) detail = "PWM µs";
    return `
      <div class="vision-rc-channel ${isPwm ? "command" : "special"} ${labelsByChannel.has(channel) ? "controlled" : ""}">
        <span>${label} · CH${channel}</span>
        <strong>${displayValue}</strong>
        <small>${detail}</small>
      </div>
    `;
  }).join("");
  $("vision-rc-meta").textContent = channels.length
    ? topicAgeText(state.vision.topics.vision_rc_command)
    : "No output";
}

function renderVisionCanvas() {
  const canvas = $("vision-canvas");
  const ctx = canvas.getContext("2d");
  const image = state.vision.frameImage;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (!image) return;

  const imageWidth = image.width || image.naturalWidth;
  const imageHeight = image.height || image.naturalHeight;
  if (!imageWidth || !imageHeight) return;
  const scale = Math.min(canvas.width / imageWidth, canvas.height / imageHeight);
  const drawWidth = imageWidth * scale;
  const drawHeight = imageHeight * scale;
  const offsetX = (canvas.width - drawWidth) / 2;
  const offsetY = (canvas.height - drawHeight) / 2;
  ctx.drawImage(image, offsetX, offsetY, drawWidth, drawHeight);
}

function showThrottledVisionError(error) {
  const now = Date.now();
  if (!state.vision.lastErrorAt || now - state.vision.lastErrorAt > 3000) {
    state.vision.lastErrorAt = now;
    showError(error);
  }
}

function showError(error) {
  const line = `${new Date().toLocaleTimeString()} ${error.message}`;
  $("log-output").textContent = `${$("log-output").textContent}\n${line}`.trim();
}

bindControls();
connectStatusSocket();
