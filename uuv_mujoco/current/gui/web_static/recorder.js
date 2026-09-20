// Recorder reset releases pilot input; recording never arms the vehicle.
(() => {
  const panel = document.createElement('fieldset');
  panel.className = 'group';
  panel.id = 'recorderPanel';
  panel.innerHTML = `<legend>VLA 시연 녹화</legend>
    <div class="rec-progress-box">
      <div class="summary-row"><strong id="recCountTitle">누적 시연 집계 대기</strong><span id="recCountPercent">—</span></div>
      <progress id="recCountProgress" max="50" value="0" aria-label="분리 성공 저장 50개 목표"></progress>
      <p id="recCountDetails" class="status-line">저장 파일을 확인하고 있습니다.</p>
      <small>분리 임무 3종 지시문 기준 · 성공 표시는 영상 검수 전입니다. 접근·정지 테스트와 다른 지시문은 기타로 분류합니다.</small>
    </div>
    <label>작업 지시문<input id="recTask" value="" style="width:100%"></label>
    <p class="status-line">새 세션마다 같은 분리 임무의 표현을 무작위 선택합니다. 세션 안에서는 지시문을 유지합니다.</p>
    <label>수집 모드 <select id="recMode"><option>STABILIZE</option><option>ALT_HOLD</option><option>MANUAL</option></select></label>
    <div class="rec-progress-box">
      <label>한 시연 목표 시간 · 시뮬 초 <input id="recTargetSeconds" type="number" min="5" max="600" step="5" value="30"></label>
      <div class="summary-row"><strong id="recProgressPercent">0%</strong><span id="recProgressTime">0.0 / 30초</span></div>
      <progress id="recProgress" max="100" value="0" aria-label="목표 녹화 시간 대비 진행률"></progress>
      <p id="recProgressHint" class="status-line">녹화 대기 · 기본 30초는 연결 점검용 분량입니다.</p>
      <small>100%는 시간 목표입니다. 분리 성공률·학습 데이터 충분도를 뜻하지 않습니다. 분리되면 먼저 저장해도 되고, 100% 이후에도 계속 녹화됩니다.</small>
    </div>
    <div id="recChecks" class="rec-checks" aria-label="수집 입력 준비 상태"></div>
    <div class="button-row"><button id="recPrepare">레코더 준비</button><button id="recStart" disabled>● 녹화 시작</button><button id="recClose" disabled>세션 종료</button></div>
    <div class="button-row"><button id="recSuccess" disabled>성공 저장</button><button id="recFailure" disabled>실패 저장</button><button id="recDiscard" disabled>현재 녹화 폐기</button></div>
    <div class="button-row"><button id="recReset" disabled>↺ 다음 시연 초기화</button></div>
    <small>저장 → 초기화 → 조종 입력 활성화 → 녹화. 로봇·부표는 시작 위치로 복구하며 ARM과 세션은 유지합니다. STABILIZE/MANUAL 지원.</small>
    <p id="recState" role="status" aria-live="polite">레코더 준비를 눌러 시작하세요.</p>
    <p id="recError" role="alert"></p><p id="recMissing"></p><p id="recResult" style="overflow-wrap:anywhere"></p>
    <details><summary>사용 순서</summary><p>카메라 VLA lite 적용 + 재시작 → 준비 → 자세 정렬·ARM·조종 입력 활성화 → 녹화 → 저장 순서입니다. 센서 출력은 10Hz 이상이 필요하며, 요청 주기와 실제 수신 주기는 다를 수 있습니다. 준비 버튼은 자동 ARM하거나 로봇을 움직이지 않습니다. 다음 시연 초기화는 저장 후 세션을 유지한 채 사용할 수 있습니다. 센서·시뮬레이션 설정 변경과 Stop/Reset은 세션 종료 후 가능합니다. 미리보기 속도는 녹화와 독립적으로 낮출 수 있습니다. 실패·자동 중단 자료는 학습용 성공 시연과 별도로 검토하세요.</p></details>`;
  document.querySelector('.control-column').appendChild(panel);
  const el = id => document.getElementById(id);
  const taskVariants = [
    'Approach the yellow buoy, align the fixed fork, and detach the buoy.',
    'Move toward the yellow buoy, align the fixed fork, and release the buoy from its attachment.',
    'Use the fixed fork to detach the yellow buoy after approaching and aligning with it.',
  ];
  function chooseTask() {
    const choices = taskVariants.filter(task => task !== el('recTask').value);
    el('recTask').value = choices[Math.floor(Math.random() * choices.length)];
  }
  chooseTask();
  let wasConfigurationLocked = false;
  let lastRecorderStatus = {};
  let recentDuration = 0;
  let recentEpisodeKey = '';
  function renderProgress(r) {
    const input = el('recTargetSeconds');
    const target = Math.min(600, Math.max(5, Number(input.value) || 30));
    const episodeKey = `${r.session_id || ''}:${r.episode_index || 0}`;
    if (r.active) {
      if (episodeKey !== recentEpisodeKey) recentDuration = 0;
      recentEpisodeKey = episodeKey;
      const duration = Number(r.duration_s);
      if (Number.isFinite(duration)) recentDuration = Math.max(0, duration);
    } else if (!r.last_result?.path) {
      recentDuration = 0;
    }
    // Saved frame count only gives an approximation after page reload (10 Hz).
    const duration = r.active ? recentDuration : r.last_result?.path
      ? Math.max(0, (Number(r.last_result.frames || 0) - 1) / 10) : 0;
    const percent = Math.min(100, Math.max(0, duration / target * 100));
    el('recProgress').value = percent;
    el('recProgressPercent').textContent = `${Math.floor(percent)}%`;
    el('recProgressTime').textContent = `${r.active ? '' : duration ? '최근 저장 약 ' : ''}${duration.toFixed(1)} / ${target}초`;
    input.disabled = Boolean(r.active || pending);
    el('recProgressHint').textContent = !r.online && r.active
      ? '상태 수신 끊김 · 마지막 확인값입니다. 녹화 상태를 확인하세요.'
      : r.active ? percent >= 100
        ? '시간 목표 도달 · 자동 종료되지 않습니다. 실제 임무 결과에 맞춰 저장하세요.'
        : `녹화 중 · 목표까지 시뮬 ${Math.max(0, target - duration).toFixed(1)}초`
      : r.last_result?.path ? '최근 저장 분량 · 다음 녹화 시작 시 0%로 초기화'
      : '녹화 대기 · 목표 시간은 시작 전에 바꿀 수 있습니다.';
  }
  el('recTargetSeconds').addEventListener('change', () => {
    el('recTargetSeconds').value = Math.min(600, Math.max(5, Number(el('recTargetSeconds').value) || 30));
    renderProgress(lastRecorderStatus);
  });
  const checks = [['ego camera','전방'],['buoy-release camera','손 카메라'],['IMU','IMU'],['depth','수심'],['RC override','RC'],['armed/mode/single RC publisher/provenance','운용 조건'],['simulation clock','시계']];
  checks.forEach(([key,label], index) => {
    const item = document.createElement('div'); item.id = 'recCheck' + index;
    const name = document.createElement('span'); name.textContent = label;
    const value = document.createElement('strong'); value.textContent = '미연결';
    item.append(name,value); el('recChecks').append(item);
  });
  let pending = false;
  async function send(payload) {
    if (pending) return;
    pending = true;
    el("recError").textContent = "";
    try {
      const response = await fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
      const body = await response.json();
      if (!response.ok || body.ok === false) throw new Error(body.error || '요청 실패');
      el('recState').textContent = body.message || '처리 중';
    } catch (e) { el('recError').textContent = e.message; }
    finally { pending = false; }
  }
  el('recReset').onclick = () => {
    window.demoResetPending = true;
    releaseInput();
    return send({command:'recorder_action', action:'reset'});
  };
  el('recPrepare').onclick = () => send({command:'recorder_prepare', task:el('recTask').value, mode:el('recMode').value});
  for (const [id,action] of [['recClose','close'],['recStart','start'],['recSuccess','success'],['recFailure','failure'],['recDiscard','discard']]) {
    el(id).onclick = () => send({command:'recorder_action',action});
  }
  const names = {'ego camera':'전방 영상', 'buoy-release camera':'손 카메라', 'depth':'수심', 'RC override':'조종 입력', 'simulation clock':'시뮬 시계', 'task instruction':'작업 지시문', 'armed/mode/single RC publisher/provenance':'ARM·선택 모드·단일 조종자·출처 정보'};
  window.renderRecorder = r => {
    const counts = r.collection_counts;
    if (counts) {
      const goal = counts.goal || 50;
      el('recCountTitle').textContent = `분리 성공 ${counts.success} / ${goal}개`;
      el('recCountPercent').textContent = `${Math.min(100, Math.floor(counts.success / goal * 100))}%`;
      el('recCountProgress').max = goal;
      el('recCountProgress').value = Math.min(goal, counts.success);
      el('recCountDetails').textContent = `전체 ${counts.total}개 · 분리 실패 ${counts.failure}개 · 기타/테스트 ${counts.other}개 · 목표까지 ${Math.max(0, goal - counts.success)}개${counts.unreadable ? ` · 읽기 오류 ${counts.unreadable}개` : ''}`;
    } else {
      el('recCountDetails').textContent = '누적 집계를 불러오려면 업데이트된 GUI 서버를 실행하세요.';
    }
    window.demoResetPending = Boolean(r.resetting);
    lastRecorderStatus = r;
    renderProgress(r);
    checks.forEach(([key], index) => {
      const cell = el('recCheck' + index), missing = (r.missing || []).includes(key);
      const value = !r.online ? '미연결' : missing ? '대기' : '확인';
      if (cell.lastElementChild.textContent !== value) cell.lastElementChild.textContent = value;
      cell.dataset.ready = String(Boolean(r.online && !missing));
    });
    const blocked = pending || r.busy || !r.online || !r.owned;
    const configurationLocked = Boolean(r.configuration_locked);
    if (wasConfigurationLocked && !configurationLocked && !r.running && !r.online) chooseTask();
    wasConfigurationLocked = configurationLocked;
    // A refreshed browser must show the recorder's real instruction, not a new draft.
    if ((configurationLocked || r.running || r.online) && r.task) el('recTask').value = r.task;
    for (const id of ['stereoCameraProfile', 'stereoCameraOptics', 'stereoCameraApplyBtn', 'stereoCameraSaveBtn',
                     'stackResetBtn', 'physicsApplyBtn', 'physicsApplyRestartBtn',
                     'courseSaveBtn', 'courseSaveResetBtn', 'toolEditorSaveBtn']) {
      const button = el(id);
      if (button) button.disabled = configurationLocked;
    }
    el('recPrepare').disabled = pending || configurationLocked || r.running || r.online;
    el('recTask').disabled = pending || configurationLocked || r.running || r.online;
    el('recMode').disabled = r.running || r.online;
    el('recReset').disabled = blocked || r.active || !['STABILIZE', 'MANUAL'].includes(r.expected_mode);
    el('recStart').disabled = blocked || r.active || !r.ready;
    el('recClose').disabled = pending || !configurationLocked || (r.running && (blocked || r.active));
    for (const id of ['recSuccess','recFailure','recDiscard']) el(id).disabled = blocked || !r.active;
    const state = !r.online ? (r.running ? '준비 중 / 상태 수신 대기' : '레코더 미연결') : r.active ? `● 녹화 중 · ${r.frames}프레임 · 시뮬 ${Number(r.duration_s || 0).toFixed(1)}초` : r.ready ? '녹화 준비 완료' : '입력 준비 필요';
    el('recState').textContent = state + ' — ' + (r.message || '');
    const overview = el('vlaOverview'), quick = el('vlaQuickOpen');
    if (overview) overview.textContent = state;
    if (quick) {
      quick.textContent = r.active ? (r.online ? `녹화 중 · ${r.frames || 0} frames` : '녹화 상태 확인 필요') : 'VLA 수집';
      quick.dataset.recording = String(Boolean(r.active));
    }
    el('recMissing').textContent = [...(r.missing || []).map(x => names[x] || x), ...(r.warnings || []).map(x => '확인: '+x)].join(' / ');
    const last = r.last_result || {};
    el('recResult').textContent = last.termination_reason ? `최근 결과: ${last.termination_reason === 'operator_stop' ? (last.success ? '성공 저장' : '실패 저장') : last.termination_reason} · ${last.frames || 0}프레임 · ${last.path || ''}` : (r.dataset_root || '');
  };
})();
