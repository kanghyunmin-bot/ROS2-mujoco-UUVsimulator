// Research workspace composition. Move existing controls to preserve command contracts.
(() => {
  const tr = window.StationLanguage.text;
  const byId = id => document.getElementById(id);
  const node = (tag, className, html = '') => {
    const element = document.createElement(tag);
    element.className = className;
    element.innerHTML = html;
    return element;
  };
  const icon = name => `<img class="ui-icon" src="/icons/feather/${name}.svg" alt="" aria-hidden="true">`;
  const main = byId('station-operation');
  const center = main.querySelector('.studio-center');
  const telemetry = byId('telemetryPanel');
  const controls = main.querySelector('.control-column');
  const camera = byId('stereoCameraPanel');
  const nav = document.querySelector('.station-nav');
  const data = byId('station-data');
  const setup = byId('station-setup');
  const shell = node('section', 'workbench');
  shell.setAttribute('aria-label', tr("Underwater Robotics Workspace", "수중 로봇 실험 워크스페이스"));
  document.querySelector('.app-header').after(shell);

  // The sidebar describes work and visible views; it is not a second command surface.
  const sidebar = node('aside', 'workspace-sidebar', `<div class="sidebar-heading">WORKSPACE</div><div class="workspace-project">${icon('layers')}<div><strong id="workspaceProject">UT27 Research</strong><small>${tr("Underwater Simulation", "수중 로봇 시뮬레이션")}</small></div></div><div class="sidebar-heading">${tr("Workspace Pages", "작업")}</div>`);
  sidebar.append(nav);
  const navIcons = ['activity', 'folder', 'sliders'];
  [...nav.children].forEach((button, index) => button.insertAdjacentHTML('afterbegin', icon(navIcons[index])));
  const views = node('div', 'workspace-views', `<div class="sidebar-heading">${tr("Camera Views", "관찰 화면")}</div><button type="button" data-camera-layout="split" aria-pressed="true" data-ut27-only>${icon('layers')}${tr("All Cameras", "전체 보기")}</button><button type="button" data-camera-layout="front" aria-pressed="false">${icon('video')}${tr("Front Camera", "전방 카메라")}</button><button type="button" data-camera-layout="hand" aria-pressed="false" data-ut27-only>${icon('video')}${tr("Hand Camera", "손 카메라")}</button><div class="sidebar-heading" data-ut27-only>${tr("Experiment Tools", "실험 도구")}</div><button type="button" id="workspacePhysics" data-ut27-only>${icon('sliders')}${tr("Physics Profile", "물리 프로필")}</button><button type="button" id="workspaceCourse" data-ut27-only>${icon('crosshair')}${tr("Initial Layout", "시작 배치")}</button>`);
  sidebar.append(views, node('div', 'sidebar-foot', `${icon('cpu')}<span>MuJoCo · ROS 2<small id="workspaceFoot">UT27 / local workspace</small></span>`));

  const stage = node('section', 'workspace-stage');
  const stageHeader = node('div', 'workspace-toolbar', `<div><span class="workspace-breadcrumb" id="workspaceBreadcrumb">UT27 / </span><strong id="workspaceTitle">${tr("Live View", "실시간 관찰")}</strong></div><div class="workspace-toolbar-actions"><span class="workspace-hint">${tr("View", "화면")}</span><button id="workspaceInspectorToggle" type="button" aria-pressed="true" aria-label="${tr("Toggle the inspector", "상세 패널 표시 전환")}">${icon('sidebar')}${tr("Inspector", "상세 패널")}</button></div>`);
  const pages = node('div', 'workspace-pages');
  pages.append(main, data, setup);
  stage.append(stageHeader, pages);
  shell.append(sidebar, stage);

  // Keep the original camera element intact so pop-out and collection can reuse it.
  const cameraLegend = camera.querySelector('legend');
  cameraLegend.textContent = tr("Cameras", "카메라");
  const visualGrid = telemetry.querySelector('.visual-grid');
  const sensor = center.querySelector('fieldset:not(#stereoCameraPanel)');
  sensor.classList.add('workspace-sensors');
  const instruments = node('div', 'workspace-instruments');
  instruments.append(...visualGrid.children, sensor);
  visualGrid.remove();
  center.append(instruments);
  main.className = 'workspace-observation';

  // Explicit empty states replace a repeated NO SIGNAL label.
  for (const [id, title, detail] of [
    ['stereoLeftView', tr("Waiting for front camera", "전방 카메라 대기 중"), tr("Start the simulation to see video.", "시뮬레이션을 시작하면 영상이 표시됩니다.")],
    ['stereoRightView', tr("Waiting for hand camera", "손 카메라 대기 중"), tr("Monitor contact between the fork and the target.", "포크와 목표물의 접촉을 확인합니다.")],
  ]) {
    byId(id).append(node('div', 'camera-empty', `${icon('video')}<strong>${title}</strong><small>${detail}</small>`));
  }

  const inspector = node('aside', 'workspace-inspector', `<div class="inspector-title"><strong>${tr("Inspector", "상세 패널")}</strong><button type="button" id="workspaceInspectorClose" aria-label="${tr("Close inspector", "상세 패널 닫기")}">${tr("Close", "닫기")}</button></div><div class="inspector-tabs" role="tablist" aria-label="${tr("Inspector tabs", "상세 패널 선택")}"><button type="button" id="inspectorControlTab" role="tab" aria-controls="inspectorControls" aria-selected="true">${tr("Controls", "조종")}</button><button type="button" id="inspectorStatusTab" role="tab" aria-controls="telemetryPanel" aria-selected="false">${tr("Vehicle Status", "기체 상태")}</button></div>`);
  const inspectorBody = node('div', 'inspector-body');
  controls.id = 'inspectorControls';
  controls.setAttribute('role', 'tabpanel');
  controls.setAttribute('aria-labelledby', 'inspectorControlTab');
  const motorStatus = node('fieldset', 'group bluerov-motors', `<legend>${tr("Thruster Output · PWM", "추진기 출력 · PWM")}</legend><div id="bluerovMotorSource" class="status-line">${tr("Waiting for output", "출력 수신 대기")}</div><div id="bluerovMotorRows" class="bluerov-motor-rows"></div>`);
  motorStatus.setAttribute('data-bluerov-only', '');
  controls.insertBefore(motorStatus, byId('pilotControlGroup'));
  telemetry.setAttribute('role', 'tabpanel');
  telemetry.setAttribute('aria-labelledby', 'inspectorStatusTab');
  telemetry.hidden = true;
  controls.querySelector('h2').remove();
  // Keep the original live controls outside the independently collapsible inspector.
  const pilot = byId('pilotControlGroup');
  const pilotDock = node('div', 'workspace-pilot');
  pilotDock.append(pilot);
  stage.append(pilotDock);
  inspectorBody.append(controls, telemetry);
  inspector.append(inspectorBody);
  const resizer = node('div', 'inspector-resizer');
  resizer.setAttribute('role', 'separator');
  resizer.setAttribute('aria-label', tr("Resize the inspector", "상세 패널 너비 조절"));
  resizer.setAttribute('aria-orientation', 'vertical');
  resizer.setAttribute('aria-valuemin', '240');
  resizer.setAttribute('aria-valuemax', '600');
  resizer.tabIndex = 0;
  inspector.prepend(resizer);
  const setInspectorWidth = (width, save = false) => {
    const max = Math.max(240, Math.min(600, window.innerWidth - 510));
    const value = Math.round(Math.max(240, Math.min(max, width)));
    shell.style.setProperty('--inspector-width', `${value}px`);
    resizer.setAttribute('aria-valuemax', String(max));
    resizer.setAttribute('aria-valuenow', String(value));
    if (save) { try { localStorage.setItem('workspaceInspectorWidth', String(value)); } catch (_) {} }
  };
  let storedWidth;
  try { storedWidth = Number(localStorage.getItem('workspaceInspectorWidth')); } catch (_) {}
  setInspectorWidth(storedWidth || 300);
  window.addEventListener('resize', () => setInspectorWidth(parseInt(resizer.getAttribute('aria-valuenow'), 10)));
  resizer.addEventListener('pointerdown', event => {
    if (event.button !== 0) return;
    event.preventDefault();
    resizer.setPointerCapture(event.pointerId);
  });
  resizer.addEventListener('pointermove', event => {
    if (resizer.hasPointerCapture(event.pointerId)) setInspectorWidth(window.innerWidth - event.clientX);
  });
  resizer.addEventListener('pointerup', event => {
    if (resizer.hasPointerCapture(event.pointerId)) {
      setInspectorWidth(window.innerWidth - event.clientX, true);
      resizer.releasePointerCapture(event.pointerId);
    }
  });
  resizer.addEventListener('keydown', event => {
    if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
    event.preventDefault();
    const current = Number(resizer.getAttribute('aria-valuenow'));
    const width = event.key === 'Home' ? 240 : event.key === 'End' ? 600 : current + (event.key === 'ArrowLeft' ? 10 : -10);
    setInspectorWidth(width, true);
  });
  inspector.querySelectorAll('[role=tab]').forEach(button => {
    button.addEventListener('click', () => {
      for (const tab of inspector.querySelectorAll('[role=tab]')) {
        const active = tab === button;
        tab.setAttribute('aria-selected', String(active));
        tab.tabIndex = active ? 0 : -1;
        byId(tab.getAttribute('aria-controls')).hidden = !active;
      }
    });
    button.addEventListener('keydown', event => {
      if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
      event.preventDefault();
      const tabs = [...inspector.querySelectorAll('[role=tab]')];
      const next = event.key === 'Home' ? tabs[0] : event.key === 'End' ? tabs[1] : tabs.find(tab => tab !== button);
      next.click(); next.focus();
    });
  });

  // This is the existing RC command replay, not a new video timeline.
  const replay = byId('rcReplayStatus').closest('fieldset');
  const dock = node('section', 'workspace-timeline');
  dock.setAttribute('aria-label', tr("RC Log Replay", "조종 명령 로그 재생"));
  replay.querySelector('legend').textContent = tr("RC Log Replay", "조종 명령 재생");
  byId('rcReplaySlider').setAttribute('aria-label', tr("RC replay position", "조종 명령 재생 위치"));
  byId('rcReplayRate').setAttribute('aria-label', tr("RC replay speed", "조종 명령 재생 배속"));
  byId('rcReplayRate').closest('label').firstChild.textContent = tr("Speed ", "배속 ");
  const replayNote = node('p', 'replay-note', tr("Replay recorded RC inputs from a rosbag in the simulator.", "기록된 RC 입력 rosbag을 시뮬레이터에 입력합니다."));
  replay.append(replayNote);
  dock.append(replay);
  shell.append(inspector, dock);
  if (window.matchMedia('(max-width: 950px)').matches) {
    shell.classList.add('inspector-collapsed');
    byId('workspaceInspectorToggle').setAttribute('aria-pressed', 'false');
  }

  // Navigation changes the workspace only; never implicitly enables RC or ARM.
  for (const button of nav.querySelectorAll('button')) {
    button.addEventListener('click', () => {
      byId('workspaceTitle').textContent = button.textContent;
      views.hidden = button.getAttribute('aria-controls') !== 'station-operation';
    });
  }
  views.querySelectorAll('[data-camera-layout]').forEach(button => button.addEventListener('click', () => {
    nav.querySelector('[aria-controls=station-operation]').click();
    camera.dataset.layout = button.dataset.cameraLayout;
    views.querySelectorAll('[data-camera-layout]').forEach(entry => entry.setAttribute('aria-pressed', String(entry === button)));
  }));
  byId('workspacePhysics').onclick = () => byId('physicsOpenBtn').click();
  byId('workspaceCourse').onclick = () => byId('courseOpenBtn').click();
  byId('workspaceInspectorToggle').onclick = event => {
    const hidden = shell.classList.toggle('inspector-collapsed');
    event.currentTarget.setAttribute('aria-pressed', String(!hidden));
  };
  byId('workspaceInspectorClose').onclick = () => byId('workspaceInspectorToggle').click();
  byId('inspectorStatusTab').tabIndex = -1;
  document.querySelector('.title-stack').innerHTML = '<h1><span id="workspaceVehicleName">UT27</span> <span>Research Studio</span></h1><p id="workspaceVehicle" hidden></p>';
  byId('settingsOpen').textContent = tr("Settings", "설정");
  byId('settingsDialog').querySelector('h2').textContent = tr("Settings", "설정");
  byId('setup-heading').textContent = tr("Experiment Setup", "실험 조건");
  byId('telemetryToggle').hidden = true;
  byId('cameraDialog').querySelector('h2').textContent = tr("Expanded Camera View", "카메라 확대");
  byId('recorderDialog').querySelector('h2').textContent = tr("Demonstration Collection", "시연 수집");
  byId('vehicleDetailsToggle').textContent = tr("Details", "상세");
  byId('controlDetailsToggle').textContent = tr("Axis Values", "축 값");

  const quick = byId('vlaQuickOpen');
  quick.setAttribute('data-ut27-only', '');
  let previousLayout = camera.dataset.layout || 'split';
  window.setWorkspaceVehicle = vehicle => {
    const blue = vehicle === 'bluerov2_standard';
    const previous = document.body.dataset.vehicle;
    document.body.dataset.vehicle = blue ? 'bluerov2_standard' : 'ut27';
    const label = blue ? 'BlueROV2 Standard' : 'UT27';
    byId('workspaceVehicleName').textContent = label;
    byId('workspaceProject').textContent = `${label} Research`;
    byId('workspaceBreadcrumb').textContent = `${label} / `;
    byId('workspaceFoot').textContent = `${label} / local workspace`;
    if (blue && previous !== 'bluerov2_standard') {
      previousLayout = camera.dataset.layout || 'split';
      camera.dataset.layout = 'front';
      byId('rcReplayPath').value = '';
      byId('rcReplayPath').placeholder = tr("RC input rosbag path", "RC 입력 rosbag 경로");
      if (!data.hidden) nav.querySelector('[aria-controls=station-operation]').click();
    } else if (!blue && previous === 'bluerov2_standard') {
      camera.dataset.layout = previousLayout;
      byId('rcReplayPath').value = '';
      byId('rcReplayPath').placeholder = tr("Enter the RC log path", "RC 로그 경로 입력");
    }
    views.querySelectorAll('[data-camera-layout]').forEach(button => {
      button.setAttribute('aria-pressed', String(button.dataset.cameraLayout === camera.dataset.layout));
    });
  };

  // Static, locally vendored Feather icons need no client-side library or network.
  const actionIcons = {
    stackStartBtn:'play', stackStopBtn:'square', stackResetBtn:'rotate-ccw', settingsOpen:'settings', quickCenterBtn:'crosshair',
    armBtn:'play', disarmBtn:'square', vlaPrepareControlBtn:'cpu', vlaRestoreManualBtn:'rotate-ccw',
    centerBtn:'crosshair', releaseBtn:'square', stereoCameraZoomBtn:'maximize-2',
    physicsOpenBtn:'sliders', courseOpenBtn:'crosshair', rcReplayLoadBtn:'folder',
    rcReplayPlayBtn:'play', rcReplayPauseBtn:'pause', rcReplayStopBtn:'square',
    vlaOpenBtn:'circle', autoCollectionQuickOpen:'layers', recPrepare:'check', recStart:'circle',
    recClose:'square', recSuccess:'check', recFailure:'x', recDiscard:'x', recReset:'rotate-ccw',
  };
  for (const [id, name] of Object.entries(actionIcons)) {
    const button = byId(id);
    if (button) button.style.setProperty('--action-icon', `url("/icons/feather/${name}.svg")`);
  }
  document.querySelectorAll('.help-info').forEach(button => {
    button.textContent = '';
    button.insertAdjacentHTML('afterbegin', icon('info'));
  });
})();
