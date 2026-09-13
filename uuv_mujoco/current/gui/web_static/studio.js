// One-time layout composition; no render loop, dependencies or 3-D engine.
(() => {
  const main = document.querySelector('main');
  const telemetry = document.getElementById('telemetryPanel');
  const controls = document.querySelector('.control-column');
  const camera = document.getElementById('stereoCameraPanel');
  const recorder = document.getElementById('recorderPanel');
  const tools = document.getElementById('toolsBody').closest('section');
  const center = document.createElement('section');
  center.className = 'studio-center';
  center.setAttribute('aria-label', '영상 및 시연 수집');
  const sensor = document.createElement('fieldset');
  sensor.className = 'group';
  sensor.innerHTML = `<legend>02 / SENSOR TELEMETRY</legend><div class="sensor-grid">
    <div class="sensor-cell"><span>IMU / ATTITUDE</span><strong id="sensorAttitude">—</strong><small id="sensorImuAge">수신 대기</small></div>
    <div class="sensor-cell"><span>DEPTH / m</span><strong id="sensorDepth">—</strong><small id="sensorDepthAge">수신 대기</small></div>
    <div class="sensor-cell"><span>VELOCITY / m·s⁻¹</span><strong id="sensorVelocity">—</strong><small id="sensorVelocitySource">수신 대기</small></div>
    <div class="sensor-cell"><span>CAMERAS / FRAME AGE</span><strong id="sensorCameras">—</strong><small>전방 / 손 · 수신 후 경과 시간</small></div>
  </div>`;
  const launch = document.createElement('fieldset');
  launch.className = 'group';
  launch.innerHTML = `<legend>03 / VLA DATA COLLECTION</legend><div class="vla-launch"><div><strong>시연 수집 워크스페이스</strong><p id="vlaOverview">별도 창에서 작업 설정 · 녹화 · 결과 확인</p></div><button id="vlaOpenBtn">VLA 수집 창 열기</button></div>`;
  center.append(camera, sensor, launch);
  const dialog = document.createElement('div');
  dialog.id = 'recorderDialog'; dialog.className = 'dialog hidden';
  dialog.innerHTML = `<div class="dialog-card recorder-dialog"><div class="summary-row"><h2>VLA / DEMONSTRATION WORKSPACE</h2><button id="vlaCloseBtn">닫기 · Esc</button></div><p class="status-line">이 창을 닫아도 녹화는 계속됩니다. 종료하려면 저장 또는 폐기를 선택하세요.</p><div id="recorderMount"></div><div class="recorder-guide"><div><strong>01 / PREPARE</strong><p>작업 지시문과 모드를 정하고 입력 준비 상태를 확인합니다.</p></div><div><strong>02 / DEMONSTRATE</strong><p>녹화 시작 후 창을 닫고 조종합니다. 상단에 녹화 상태가 유지됩니다.</p></div><div><strong>03 / REVIEW</strong><p>창을 다시 열어 성공·실패를 저장하거나 현재 시연을 폐기합니다.</p></div></div></div>`;
  document.body.append(dialog);
  const cameraDialog = document.createElement('div');
  cameraDialog.id = 'cameraDialog'; cameraDialog.className = 'dialog hidden';
  cameraDialog.innerHTML = '<div class="dialog-card camera-dialog"><h2>CAMERA / LIVE VIEW</h2><div id="cameraWindowBody"></div></div>';
  document.body.append(cameraDialog);
  dialog.querySelector('#recorderMount').append(recorder);
  const quick = document.createElement('button'); quick.id = 'vlaQuickOpen'; quick.textContent = 'VLA 수집';
  document.querySelector('.header-actions').append(quick);
  for (const button of [quick, launch.querySelector('button')]) button.onclick = () => window.StationWindows.open('recorderDialog');
  document.getElementById('vlaCloseBtn').onclick = () => window.StationWindows.close('recorderDialog');
  const advanced = document.createElement('details');
  advanced.className = 'studio-advanced';
  const summary = document.createElement('summary');
  summary.textContent = '고급 도구 / 재생 · 물리 · 환경 설정';
  advanced.append(summary, tools);
  // Move existing controls, preserving IDs, event targets and live camera nodes.
  main.replaceChildren(telemetry, center, controls, advanced);
  main.className = 'studio-grid';
  controls.querySelector('h2').textContent = '04 / VEHICLE CONTROL';
  for (const [id,label] of Object.entries({stackStartBtn:'시뮬 시작',stackResetBtn:'정지 / 리셋',armBtn:'ARM',disarmBtn:'DISARM',quickCenterBtn:'스틱 중립',quickPhysicsOpenBtn:'물리 설정',telemetryToggle:'상태 패널 접기'})) {
    const button = document.getElementById(id);
    if(button) button.textContent = label;
  }
  // Keep fieldset semantics but collapse secondary information by default.
  for(const id of ['rcFeedbackBars','eventList']) {
    const target = document.getElementById(id).closest('fieldset');
    const details = document.createElement('details');
    const title = document.createElement('summary');
    title.textContent = id === 'eventList' ? '이벤트 로그' : 'RC 채널 피드백';
    target.before(details); details.append(title,target);
  }
})();
