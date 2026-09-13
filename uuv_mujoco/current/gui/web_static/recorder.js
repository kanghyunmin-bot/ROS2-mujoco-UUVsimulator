// Recorder commands use the existing collector; this panel never changes RC.
(() => {
  const panel = document.createElement('fieldset');
  panel.className = 'group';
  panel.id = 'recorderPanel';
  panel.innerHTML = `<legend>VLA 시연 녹화</legend>
    <label>작업 지시문<input id="recTask" value="Approach the buoy and stop in front of it." style="width:100%"></label>
    <label>수집 모드 <select id="recMode"><option>STABILIZE</option><option>ALT_HOLD</option><option>MANUAL</option></select></label>
    <div id="recChecks" class="rec-checks" aria-label="수집 입력 준비 상태"></div>
    <div class="button-row"><button id="recPrepare">레코더 준비</button><button id="recStart" disabled>● 녹화 시작</button><button id="recClose" disabled>세션 종료</button></div>
    <div class="button-row"><button id="recSuccess" disabled>성공 저장</button><button id="recFailure" disabled>실패 저장</button><button id="recDiscard" disabled>현재 녹화 폐기</button></div>
    <p id="recState" role="status" aria-live="polite">레코더 준비를 눌러 시작하세요.</p>
    <p id="recError" role="alert"></p><p id="recMissing"></p><p id="recResult" style="overflow-wrap:anywhere"></p>
    <details><summary>사용 순서</summary><p>준비 → 자세 정렬·ARM·조종 입력 활성화 → 녹화 → 저장 순서입니다. 준비 버튼은 자동 ARM하거나 로봇을 움직이지 않습니다. 복귀·리셋은 녹화 종료 후 하세요. 카메라가 4Hz라면 수집용으로 변경하고 센서 상태를 먼저 확인하세요. 실패·자동 중단 자료는 학습용 성공 시연과 별도로 검토하세요.</p></details>`;
  document.querySelector('.control-column').appendChild(panel);
  const el = id => document.getElementById(id);
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
  el('recPrepare').onclick = () => send({command:'recorder_prepare', task:el('recTask').value, mode:el('recMode').value});
  for (const [id,action] of [['recClose','close'],['recStart','start'],['recSuccess','success'],['recFailure','failure'],['recDiscard','discard']]) {
    el(id).onclick = () => send({command:'recorder_action',action});
  }
  const names = {'ego camera':'전방 영상', 'buoy-release camera':'손 카메라', 'depth':'수심', 'RC override':'조종 입력', 'simulation clock':'시뮬 시계', 'task instruction':'작업 지시문', 'armed/mode/single RC publisher/provenance':'ARM·선택 모드·단일 조종자·출처 정보'};
  window.renderRecorder = r => {
    checks.forEach(([key], index) => {
      const cell = el('recCheck' + index), missing = (r.missing || []).includes(key);
      const value = !r.online ? '미연결' : missing ? '대기' : '확인';
      if (cell.lastElementChild.textContent !== value) cell.lastElementChild.textContent = value;
      cell.dataset.ready = String(Boolean(r.online && !missing));
    });
    const blocked = pending || r.busy || !r.online || !r.owned;
    el('recPrepare').disabled = pending || r.running || r.online;
    el('recTask').disabled = r.running || r.online;
    el('recMode').disabled = r.running || r.online;
    el('recStart').disabled = blocked || r.active || !r.ready;
    el('recClose').disabled = blocked || r.active || !r.running;
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
