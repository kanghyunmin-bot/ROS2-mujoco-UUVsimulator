// Visual action semantics and contextual help. No command or polling endpoints.
(() => {
  const actions = {
    stackStartBtn: ['primary', '▶', '선택한 실험 프로필로 SITL과 MuJoCo를 시작합니다.'],
    stackStopBtn: ['danger', '■', '이 GUI에서 시작한 시뮬레이터를 정지합니다.'],
    stackResetBtn: ['warning', '↻', 'SITL을 정지하고 저장된 SITL 설정을 초기화합니다.'],
    quickCenterBtn: ['neutral', '⊙', '조종 축을 중립으로 보냅니다. 조종 입력 해제나 DISARM은 별도 동작입니다.'],
    settingsOpen: ['neutral', '', '표시 성능, 조종 장치, 연결 진단과 대회 도구를 설정합니다.'],
    armBtn: ['warning', '⚠', '추진기 구동을 허용합니다. 조종 입력에 따라 기체가 움직일 수 있습니다.'],
    disarmBtn: ['danger', '■', '기체를 DISARM하여 추진기 구동을 해제합니다.'],
    vlaPrepareControlBtn: ['warning', '⇄', '수동 조종 입력을 해제하고 정책 제어권을 준비합니다. 정책 실행과 ARM은 별도입니다.'],
    vlaRestoreManualBtn: ['neutral', '↩', '정책 제어 준비 상태에서 수동 조종 경로로 복귀합니다.'],
    centerBtn: ['neutral', '⊙', '전후·좌우·상하·회전 입력을 중립으로 돌립니다.'],
    releaseBtn: ['danger', '■', '브라우저의 수동 RC 입력을 해제합니다.'],
    stereoCameraApplyBtn: ['warning', '↻', '카메라 센서 출력 설정을 저장하고 시뮬레이션을 재시작합니다. 녹화 세션 중에는 사용할 수 없습니다.'],
    stereoCameraSaveBtn: ['primary', '✓', '센서 출력 설정을 저장합니다. 다음 실행부터 적용됩니다.'],
    physicsApplyBtn: ['primary', '✓', '편집한 물리값을 프로필 파일에 저장합니다.'],
    physicsApplyRestartBtn: ['warning', '↻', '물리값을 저장하고 시뮬레이션을 재시작합니다.'],
    courseSaveBtn: ['primary', '✓', '로봇·부표 배치를 파일에 저장합니다. 실행 반영에는 재시작이 필요합니다.'],
    courseSaveResetBtn: ['warning', '↻', '배치를 저장하고 시뮬레이션을 재시작합니다.'],
    courseUpdateSelectedBtn: ['neutral', '↔', '선택 물체의 편집 좌표를 갱신합니다. 배치 저장 전에는 파일이 바뀌지 않습니다.'],
    physicsOpenBtn: ['neutral', '⚙', '현재 실험 프로필의 물리값과 활성 항목을 확인·편집합니다.'],
    courseOpenBtn: ['neutral', '▦', '평면도에서 로봇과 부표의 시작 위치를 편집합니다.'],
    toolEditorSaveBtn: ['warning', '✓', '원본 설정 파일을 저장합니다. 변경 전에 파일 경로와 내용을 확인하세요.'],
    rcReplayLoadBtn: ['neutral', '↓', '입력한 경로의 RC 기록을 불러옵니다.'],
    rcReplayPlayBtn: ['primary', '▶', '기록된 조종 명령을 재생합니다. 기체가 움직일 수 있습니다.'],
    rcReplayPauseBtn: ['neutral', 'Ⅱ', 'RC 기록 재생을 일시 정지합니다.'],
    rcReplayStopBtn: ['danger', '■', 'RC 기록 재생을 중지합니다.'],
    vlaOpenBtn: ['primary', '◉', '영상과 조종기를 함께 보며 시연을 녹화하고 결과를 저장합니다.'],
    autoCollectionQuickOpen: ['neutral', '⇥', '자동 수집 설정으로 이동합니다. 여기서는 수집을 시작하지 않습니다.'],
    autoCollectionStart: ['warning', '▶', '시뮬레이터를 자동 ARM하고 FSM 조종·녹화·초기화를 반복합니다. 기체가 움직입니다.'],
    autoCollectionStop: ['danger', '■', '자동 수집을 중지합니다. 진행 중 시연은 실패로 저장하고 ARM을 해제합니다.'],
    recPrepare: ['primary', '✓', '지시문과 설정을 고정하고 레코더 입력 준비 상태를 확인합니다. 자동 ARM은 하지 않습니다.'],
    recStart: ['primary', '●', '현재 센서·조종 입력의 시연 녹화를 시작합니다.'],
    recClose: ['neutral', '■', '현재 수집 세션을 종료하고 설정 잠금을 풉니다.'],
    recSuccess: ['primary', '✓', '현재 시연을 성공 라벨로 저장합니다. 실제 임무 결과를 보고 선택하세요.'],
    recFailure: ['neutral', '×', '현재 시연을 실패 라벨로 저장합니다.'],
    recDiscard: ['danger', '×', '현재 시연을 폐기합니다. 이후 데이터 검토가 필요하면 실패 저장을 사용하세요.'],
    recReset: ['warning', '↻', '로봇과 부표를 시작 위치로 복구합니다. 세션과 ARM은 유지됩니다.'],
    rosBuildBtn: ['warning', '⚒', 'ROS 패키지를 빌드합니다. 일반적인 실험 실행 때마다 수행할 필요는 없습니다.'],
    pingConfigBtn: ['warning', '✓', '선택한 스캔 범위·게인·주파수를 소나에 전달합니다.'],
    pingerHomingStartBtn: ['warning', '▶', '대회용 비콘 추적을 시작합니다. Auto arm이 선택되어 있으면 자동 무장할 수 있습니다.'],
    pingerHomingStopBtn: ['danger', '■', '비콘 추적을 중지합니다.'],
  };
  for (const [id, [tone, icon, text]] of Object.entries(actions)) {
    const button = document.getElementById(id);
    if (!button) continue;
    button.dataset.tone = tone;
    button.dataset.icon = icon;
    button.dataset.help = text;
  }
  const fields = {
    stereoCameraPreviewHz: '이 브라우저에서 실제 표시하는 영상 속도입니다. 자동은 표시 지연에 따라 2·4·10 fps 상한을 조정합니다. 녹화·ROS 센서·물리 주기는 변경하지 않습니다.',
    stereoCameraProfile: 'ROS 카메라의 해상도와 발행 주기입니다. 녹화와 정책 입력에 영향을 주므로 실험 조건에 포함됩니다.',
    stereoCameraOptics: '카메라 렌더링의 수중 광학 설정입니다. 실험 영상에 영향을 줍니다.',
    simLaunchPreset: '장면과 호환되는 물리 프로필을 함께 선택합니다. 실행 중에는 변경할 수 없습니다.',
    sensorErrorMode: '센서 오차 모델을 선택합니다. 수학적 오차의 계수는 실측 검증값과 구분해야 합니다.',
    legacyCourseEnabled: '과거 대회 코스를 맵 편집기에서 선택할 수 있게 합니다. 연구 수영장과 보정 수조는 기본 제공됩니다.',
    recTask: '실험 지시문입니다. 녹화 세션 안에서는 고정되며 자동으로 다른 문장으로 바뀌지 않습니다.',
    recTargetSeconds: '진행률 표시용 시뮬레이션 시간 목표입니다. 100%가 되어도 녹화가 자동 종료되지는 않습니다.',
    gamepadDevice: '실제로 연결된 게임패드를 선택합니다. 장치가 끊겨도 다른 게임패드로 자동 전환하지 않습니다.',
    fcuUrlInput: 'MAVROS가 사용할 비행제어기 연결 주소입니다. 연결이 안 될 때 확인하세요.',
  };
  for (const [id, text] of Object.entries(fields)) {
    const input = document.getElementById(id);
    if (!input) continue;
    const label = input.closest('label');
    const info = document.createElement('button');
    info.type = 'button';
    info.className = 'help-info';
    const name = label?.querySelector('span')?.textContent || label?.firstChild?.textContent?.trim() || id;
    info.setAttribute('aria-label', name + ' 도움말');
    info.textContent = 'ⓘ';
    info.dataset.help = text;
    info.dataset.helpToggle = 'true';
    // Keep an interactive help control outside the form's label.
    const row = document.createElement('div');
    row.className = 'field-with-help';
    const field = label || input;
    field.before(row);
    row.append(field, info);
  }
  const tooltip = document.createElement('div');
  tooltip.id = 'stationHelp';
  tooltip.role = 'tooltip';
  tooltip.hidden = true;
  document.body.append(tooltip);
  let owner = null;
  let previousDescription = null;
  let closeTimer = null;
  function hide() {
    clearTimeout(closeTimer);
    if (owner) {
      if (previousDescription === null) owner.removeAttribute('aria-describedby');
      else owner.setAttribute('aria-describedby', previousDescription);
    }
    tooltip.hidden = true;
    owner = null;
  }
  function show(element) {
    hide();
    owner = element;
    previousDescription = element.getAttribute('aria-describedby');
    element.setAttribute('aria-describedby', [previousDescription, tooltip.id].filter(Boolean).join(' '));
    tooltip.textContent = element.dataset.help;
    tooltip.hidden = false;
    const rect = element.getBoundingClientRect();
    const box = tooltip.getBoundingClientRect();
    tooltip.style.left = Math.max(8, Math.min(rect.left, innerWidth - box.width - 8)) + 'px';
    tooltip.style.top = Math.max(8, rect.bottom + box.height + 12 < innerHeight
      ? rect.bottom + 8 : rect.top - box.height - 8) + 'px';
  }
  for (const event of ['pointerover', 'focusin']) {
    document.addEventListener(event, e => {
      const target = e.target.closest('[data-help]');
      if (target === owner) clearTimeout(closeTimer);
      else if (target) show(target);
      if (tooltip.contains(e.target)) clearTimeout(closeTimer);
    });
  }
  for (const event of ['pointerout', 'focusout']) {
    document.addEventListener(event, e => {
      if (!owner || (!owner.contains(e.target) && !tooltip.contains(e.target))) return;
      if (owner.contains(e.relatedTarget) || tooltip.contains(e.relatedTarget)) return;
      clearTimeout(closeTimer);
      closeTimer = setTimeout(hide, 200);
    });
  }
  document.addEventListener('click', e => {
    const info = e.target.closest('[data-help-toggle]');
    if (info) show(info);
    else hide();
  });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape' && !tooltip.hidden) {
      hide();
      e.preventDefault();
      e.stopImmediatePropagation();
    }
  }, true);
  document.addEventListener('scroll', hide, true);
  window.addEventListener('resize', hide);
})();
