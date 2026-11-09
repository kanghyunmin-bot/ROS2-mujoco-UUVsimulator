// Recorder reset releases pilot input; recording never arms the vehicle.
(() => {
  const t = (english, korean) => window.StationLanguage?.text(english, korean) ?? english;
  const panel = document.createElement('fieldset');
  panel.className = 'group';
  panel.id = 'recorderPanel';
  panel.innerHTML = `<legend>${t('VLA demonstration recording', 'VLA 시연 녹화')}</legend>
    <div class="rec-progress-box">
      <strong>${t('Saved demonstrations', '저장한 시연')}</strong>
      <p id="recCountDetails" class="status-line">${t('Checking saved files.', '저장 파일을 확인하고 있습니다.')}</p>
      <small>${t('Three detachment task instructions are counted. Success labels have not been video reviewed. Other instructions and approach or stop tests are counted separately.', '분리 임무 3종 지시문 기준 · 성공 표시는 영상 검수 전입니다. 접근·정지 테스트와 다른 지시문은 기타로 분류합니다.')}</small>
    </div>
    <div class="rec-progress-box" id="autoCollectionPanel">
      <strong>${t('Automatic collection · simulator', '자동 수집 모드 · 시뮬레이터')}</strong>
      <p>${t('STABILIZE · ARM → FSM control and recording → Save result → Reset map', 'STABILIZE·ARM → FSM 조종·녹화 → 성공/실패 저장 → 맵 초기화')}</p>
      <label>${t('Episodes', '수집 횟수')} <input id="autoCollectionEpisodes" type="number" min="1" max="10000" step="1" value="3"></label>
      <div class="button-row"><button id="autoCollectionStart">${t('Start automatic collection', '자동 수집 시작')}</button><button id="autoCollectionStop" disabled>${t('Stop automatic collection', '자동 수집 중지')}</button></div>
      <p id="autoCollectionState" role="status" aria-live="polite">${t('Ready · Starting will automatically arm and move the vehicle.', '대기 · 시작하면 자동으로 ARM하고 움직입니다.')}</p>
      <small>${t('Results use fork contact and release state. Stopping saves the current recording as a failure and disarms the vehicle.', '포크 접촉·고정 해제로 결과를 판정합니다. 중지 시 현재 녹화는 실패로 저장하고 ARM을 해제합니다.')}</small>
    </div>
    <label>${t('Task instruction', '작업 지시문')}<input id="recTask" value="" style="width:100%"></label>
    <p class="status-line">${t('Review or edit the task instruction. The instruction is fixed for the session.', '실험에 사용할 지시문을 확인하거나 수정하세요. 세션 안에서는 같은 지시문을 사용합니다.')}</p>
    <label>${t('Collection mode', '수집 모드')} <select id="recMode"><option>STABILIZE</option><option>ALT_HOLD</option><option>MANUAL</option></select></label>
    <div class="rec-progress-box">
      <label>${t('Target duration per demonstration · simulation seconds', '한 시연 목표 시간 · 시뮬 초')} <input id="recTargetSeconds" type="number" min="5" max="600" step="5" value="30"></label>
      <div class="summary-row"><strong id="recProgressPercent">0%</strong><span id="recProgressTime">${t('0.0 / 30 s', '0.0 / 30초')}</span></div>
      <progress id="recProgress" max="100" value="0" aria-label="${t('Recording progress toward target duration', '목표 녹화 시간 대비 진행률')}"></progress>
      <p id="recProgressHint" class="status-line">${t('Ready to record · The default 30 seconds is for checking connections.', '녹화 대기 · 기본 30초는 연결 점검용 분량입니다.')}</p>
      <small>${t('100% is the duration target. It does not indicate task success or dataset sufficiency. You may save earlier, and recording continues past 100%.', '100%는 시간 목표입니다. 분리 성공률·학습 데이터 충분도를 뜻하지 않습니다. 분리되면 먼저 저장해도 되고, 100% 이후에도 계속 녹화됩니다.')}</small>
    </div>
    <div id="recChecks" class="rec-checks" aria-label="${t('Collection input readiness', '수집 입력 준비 상태')}"></div>
    <div class="button-row"><button id="recPrepare">${t('Prepare recorder', '레코더 준비')}</button><button id="recStart" disabled>${t('● Start recording', '● 녹화 시작')}</button><button id="recClose" disabled>${t('Close session', '세션 종료')}</button></div>
    <div class="button-row"><button id="recSuccess" disabled>${t('Save success', '성공 저장')}</button><button id="recFailure" disabled>${t('Save failure', '실패 저장')}</button><button id="recDiscard" disabled>${t('Discard recording', '현재 녹화 폐기')}</button></div>
    <div class="button-row"><button id="recReset" disabled>${t('↺ Reset for next demonstration', '↺ 다음 시연 초기화')}</button></div>
    <small>${t('Save → Reset → Enable pilot input → Record. The robot and buoy return to their initial positions while arming and the session remain active. STABILIZE and MANUAL are supported.', '저장 → 초기화 → 조종 입력 활성화 → 녹화. 로봇·부표는 시작 위치로 복구하며 ARM과 세션은 유지합니다. STABILIZE/MANUAL 지원.')}</small>
    <p id="recState" role="status" aria-live="polite">${t('Select Prepare recorder to begin.', '레코더 준비를 눌러 시작하세요.')}</p>
    <p id="recError" role="alert"></p><p id="recMissing"></p><p id="recResult" style="overflow-wrap:anywhere"></p>
    <details><summary>${t('Instructions', '사용 순서')}</summary><p>${t('Apply VLA lite camera settings and restart, prepare, align the vehicle, arm and enable pilot input, record, then save. Sensors must publish at least 10 Hz. Actual rates may differ from requested rates. Prepare does not arm or move the robot. Reset for the next demonstration after saving. Close the session before changing sensor or simulation settings or using Stop/Reset. Preview rate can be lowered independently of recording. Review failed and automatically interrupted recordings separately from successful training examples.', '카메라 VLA lite 적용 + 재시작 → 준비 → 자세 정렬·ARM·조종 입력 활성화 → 녹화 → 저장 순서입니다. 센서 출력은 10Hz 이상이 필요하며, 요청 주기와 실제 수신 주기는 다를 수 있습니다. 준비 버튼은 자동 ARM하거나 로봇을 움직이지 않습니다. 다음 시연 초기화는 저장 후 세션을 유지한 채 사용할 수 있습니다. 센서·시뮬레이션 설정 변경과 Stop/Reset은 세션 종료 후 가능합니다. 미리보기 속도는 녹화와 독립적으로 낮출 수 있습니다. 실패·자동 중단 자료는 학습용 성공 시연과 별도로 검토하세요.')}</p></details>`;
  document.querySelector('.control-column').appendChild(panel);
  const el = id => document.getElementById(id);
  el('recTask').value = 'Approach the yellow buoy, align the fixed fork, and detach the buoy.';
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
    el('recProgressTime').textContent = `${r.active ? '' : duration ? t('Last saved: approx. ', '최근 저장 약 ') : ''}${duration.toFixed(1)} / ${target} ${t('s', '초')}`;
    input.disabled = Boolean(r.active || pending);
    el('recProgressHint').textContent = !r.online && r.active
      ? t('Status connection lost · showing last known value. Check the recording.', '상태 수신 끊김 · 마지막 확인값입니다. 녹화 상태를 확인하세요.')
      : r.active ? percent >= 100
        ? t('Target reached · recording continues until saved according to the actual task result.', '시간 목표 도달 · 자동 종료되지 않습니다. 실제 임무 결과에 맞춰 저장하세요.')
        : t(`Recording · ${Math.max(0, target - duration).toFixed(1)} simulation seconds to target`, `녹화 중 · 목표까지 시뮬 ${Math.max(0, target - duration).toFixed(1)}초`)
      : r.last_result?.path ? t('Last saved duration · resets to 0% for the next recording', '최근 저장 분량 · 다음 녹화 시작 시 0%로 초기화')
      : t('Ready · target duration can be changed before recording.', '녹화 대기 · 목표 시간은 시작 전에 바꿀 수 있습니다.');
  }
  el('recTargetSeconds').addEventListener('change', () => {
    el('recTargetSeconds').value = Math.min(600, Math.max(5, Number(el('recTargetSeconds').value) || 30));
    renderProgress(lastRecorderStatus);
  });
  const checks = [['ego camera',t('Front camera','전방')],['buoy-release camera',t('Hand camera','손 카메라')],['IMU','IMU'],['depth',t('Depth','수심')],['RC override','RC'],['armed/mode/single RC publisher/provenance',t('Operating conditions','운용 조건')],['simulation clock',t('Clock','시계')]];
  checks.forEach(([key,label], index) => {
    const item = document.createElement('div'); item.id = 'recCheck' + index;
    const name = document.createElement('span'); name.textContent = label;
    const value = document.createElement('strong'); value.textContent = t('Disconnected', '미연결');
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
      if (!response.ok || body.ok === false) throw new Error(body.error || t('Request failed', '요청 실패'));
      el('recState').textContent = body.message || t('Processing', '처리 중');
    } catch (e) { el('recError').textContent = e.message; }
    finally { pending = false; }
  }
  el('recReset').onclick = () => {
    window.demoResetPending = true;
    releaseInput();
    return send({command:'recorder_action', action:'reset'});
  };
  el('autoCollectionStart').onclick = () => send({command:'auto_collection_start', episodes:Number(el('autoCollectionEpisodes').value)});
  el('autoCollectionStop').onclick = () => send({command:'auto_collection_stop'});
  el('recPrepare').onclick = () => send({command:'recorder_prepare', task:el('recTask').value, mode:el('recMode').value});
  for (const [id,action] of [['recClose','close'],['recStart','start'],['recSuccess','success'],['recFailure','failure'],['recDiscard','discard']]) {
    el(id).onclick = () => send({command:'recorder_action',action});
  }
  const names = {'ego camera':t('Front camera', '전방 영상'), 'buoy-release camera':t('Hand camera', '손 카메라'), 'depth':t('Depth', '수심'), 'RC override':t('Pilot input', '조종 입력'), 'simulation clock':t('Simulation clock', '시뮬 시계'), 'task instruction':t('Task instruction', '작업 지시문'), 'armed/mode/single RC publisher/provenance':t('ARM · selected mode · single pilot · provenance', 'ARM·선택 모드·단일 조종자·출처 정보')};
  window.renderRecorder = r => {
    const counts = r.collection_counts;
    if (counts) {
      el('recCountDetails').textContent = t(`Total ${counts.total} · success ${counts.success} · failure ${counts.failure} · other/test ${counts.other}${counts.unreadable ? ` · unreadable ${counts.unreadable}` : ''}`, `전체 ${counts.total}개 · 분리 성공 라벨 ${counts.success}개 · 분리 실패 ${counts.failure}개 · 기타/테스트 ${counts.other}개${counts.unreadable ? ` · 읽기 오류 ${counts.unreadable}개` : ''}`);
    } else {
      el('recCountDetails').textContent = t('Run the updated GUI server to load collection totals.', '누적 집계를 불러오려면 업데이트된 GUI 서버를 실행하세요.');
    }
    window.demoResetPending = Boolean(r.resetting);
    lastRecorderStatus = r;
    renderProgress(r);
    checks.forEach(([key], index) => {
      const cell = el('recCheck' + index), missing = (r.missing || []).includes(key);
      const value = !r.online ? t('Disconnected', '미연결') : missing ? t('Waiting', '대기') : t('Ready', '확인');
      if (cell.lastElementChild.textContent !== value) cell.lastElementChild.textContent = value;
      cell.dataset.ready = String(Boolean(r.online && !missing));
    });
    const automatic = r.auto_collection || {};
    const autoRunning = Boolean(automatic.running);
    window.autoCollectionRunning = autoRunning;
    const blocked = autoRunning || pending || r.busy || !r.online || !r.owned;
    const configurationLocked = autoRunning || Boolean(r.configuration_locked);
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
    el('recMode').disabled = autoRunning || r.running || r.online;
    el('recReset').disabled = blocked || r.active || !['STABILIZE', 'MANUAL'].includes(r.expected_mode);
    el('recStart').disabled = blocked || r.active || !r.ready;
    el('recClose').disabled = autoRunning || pending || !configurationLocked || (r.running && (blocked || r.active));
    for (const id of ['recSuccess','recFailure','recDiscard']) el(id).disabled = blocked || !r.active;
    const state = !r.online ? (r.running ? t('Preparing / waiting for status', '준비 중 / 상태 수신 대기') : t('Recorder disconnected', '레코더 미연결')) : r.active ? t(`● Recording · ${r.frames} frames · simulation ${Number(r.duration_s || 0).toFixed(1)} s`, `● 녹화 중 · ${r.frames}프레임 · 시뮬 ${Number(r.duration_s || 0).toFixed(1)}초`) : r.ready ? t('Ready to record', '녹화 준비 완료') : t('Inputs not ready', '입력 준비 필요');
    el('recState').textContent = state + ' — ' + (r.message || '');
    const overview = el('vlaOverview'), quick = el('vlaQuickOpen');
    if (overview) overview.textContent = state;
    if (quick) {
      quick.textContent = r.active ? (r.online ? t(`Recording · ${r.frames || 0} frames`, `녹화 중 · ${r.frames || 0} frames`) : t('Check recording status', '녹화 상태 확인 필요')) : t('VLA collection', 'VLA 수집');
      quick.dataset.recording = String(Boolean(r.active));
    }
    el('recMissing').textContent = [...(r.missing || []).map(x => names[x] || x), ...(r.warnings || []).map(x => t('Check: ', '확인: ')+x)].join(' / ');
    el('autoCollectionEpisodes').disabled = autoRunning || pending;
    el('autoCollectionStart').disabled = pending || autoRunning || Boolean(r.configuration_locked);
    el('autoCollectionStop').disabled = pending || !autoRunning || automatic.stopping;
    const phases = {waiting_simulation:t('Waiting for simulator', '시뮬 연결 대기'), preparing:t('Preparing', '준비 중'), recording:t('Recording', '녹화 중'), resetting:t('Resetting map', '맵 초기화 중'), finalizing:t('Finalizing', '종료 정리 중'), complete:t('Collection complete', '수집 완료'), stopped:t('Stopped', '중지 완료'), error:t('Error', '오류')};
    const results = automatic.results || [];
    const successes = results.filter(x => x.success).length;
    el('autoCollectionState').textContent = automatic.phase || autoRunning
      ? t(`${automatic.stopping ? 'Stopping and saving' : phases[automatic.phase] || 'Starting'} · ${results.length}/${automatic.episodes || el('autoCollectionEpisodes').value} episodes · success ${successes} / failure ${results.length-successes}${automatic.error ? ' · '+automatic.error : ''}`, `${automatic.stopping ? '중지·저장 중' : phases[automatic.phase] || '시작 중'} · ${results.length}/${automatic.episodes || el('autoCollectionEpisodes').value}회 · 성공 ${successes} / 실패 ${results.length-successes}${automatic.error ? ' · '+automatic.error : ''}`)
      : t('Ready · Starting will automatically arm and move the vehicle.', '대기 · 시작하면 자동으로 ARM하고 움직입니다.');
    const last = r.last_result || {};
    el('recResult').textContent = last.termination_reason ? t(`Latest result: ${last.termination_reason === 'operator_stop' ? (last.success ? 'saved success' : 'saved failure') : last.termination_reason} · ${last.frames || 0} frames · ${last.path || ''}`, `최근 결과: ${last.termination_reason === 'operator_stop' ? (last.success ? '성공 저장' : '실패 저장') : last.termination_reason} · ${last.frames || 0}프레임 · ${last.path || ''}`) : (r.dataset_root || '');
  };
})();
