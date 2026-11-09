// Manage the offline vehicle-build job without changing the live SITL controls.
(() => {
  const tr = window.StationLanguage.text;
  const byId = id => document.getElementById(id);
  const state = { running: false, pending: false, wasRunning: false, polling: false };
  const packageInput = byId('real2simPackage');
  const nameInput = byId('real2simOutputName');
  const message = byId('real2simBuildMessage');

  function updateControls() {
    const folder = packageInput.value.trim();
    const name = nameInput.value.trim();
    const relativeFolder = folder && !folder.startsWith('/') && !folder.split('/').includes('..');
    const validName = /^[A-Za-z0-9][A-Za-z0-9_-]*$/.test(name);
    byId('real2simBuildStart').disabled = state.running || state.pending || !relativeFolder || !validName;
    byId('real2simBuildStop').disabled = !state.running || state.pending;
    packageInput.disabled = state.running || state.pending;
    nameInput.disabled = state.running || state.pending;
    if (!state.running && !state.pending && name && !validName) {
      message.textContent = tr('Output name may contain letters, numbers, - and _.', '결과 이름에는 영문, 숫자, - 및 _를 사용할 수 있습니다.');
    }
  }

  function showStatus(result) {
    const previouslyRunning = state.wasRunning;
    state.running = Boolean(result.running);
    state.wasRunning = state.running;
    byId('real2simBuildStatus').textContent = result.status || tr('Ready', '대기 중');
    byId('real2simBuildLog').textContent = typeof result.log === 'string' ? result.log : '';
    byId('real2simBuildOutput').textContent = result.output
      ? `${tr('Output', '결과')} · ${result.output}` : '';
    if (previouslyRunning && !state.running) {
      message.textContent = result.success
        ? tr('Calibration completed. The vehicle list has been refreshed.', '자동 보정이 끝났습니다. 기체 목록을 새로고침했습니다.')
        : tr('Calibration stopped or failed. Check the log.', '보정이 중지되었거나 실패했습니다. 실행 기록을 확인하세요.');
      if (typeof refreshReal2SimVehicles === 'function') refreshReal2SimVehicles();
    }
    updateControls();
  }

  async function poll() {
    if (state.polling) return;
    state.polling = true;
    try {
      const response = await fetch('/api/real2sim/build/status', { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      showStatus(await response.json());
    } catch (error) {
      byId('real2simBuildStatus').textContent = tr(`Status unavailable: ${error.message}`, `상태를 불러오지 못했습니다: ${error.message}`);
    } finally {
      state.polling = false;
    }
  }

  async function send(path, payload) {
    const response = await fetch(path, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload),
    });
    const result = await response.json().catch(() => ({}));
    if (!response.ok || result.ok === false) throw new Error(result.error || `HTTP ${response.status}`);
    return result;
  }

  async function start(event) {
    event.preventDefault();
    if (byId('real2simBuildStart').disabled) return;
    state.pending = true;
    updateControls();
    try {
      await send('/api/real2sim/build', { package: packageInput.value.trim(), output_name: nameInput.value.trim() });
      state.running = true;
      state.wasRunning = true;
      message.textContent = tr('Calibration started.', '자동 보정을 시작했습니다.');
    } catch (error) {
      message.textContent = tr(`Start failed: ${error.message}`, `시작 실패: ${error.message}`);
    } finally {
      state.pending = false;
      updateControls();
      await poll();
    }
  }

  async function stop() {
    if (!state.running || state.pending) return;
    state.pending = true;
    updateControls();
    try {
      await send('/api/real2sim/build/stop', {});
      message.textContent = tr('Stopping calibration…', '자동 보정을 중지하는 중…');
    } catch (error) {
      message.textContent = tr(`Stop failed: ${error.message}`, `중지 실패: ${error.message}`);
    } finally {
      state.pending = false;
      await poll();
      updateControls();
    }
  }

  byId('real2simBuildForm').addEventListener('submit', start);
  byId('real2simBuildStop').addEventListener('click', stop);
  packageInput.addEventListener('input', updateControls);
  nameInput.addEventListener('input', updateControls);
  updateControls();
  poll();
  setInterval(poll, 2500);
})();
