// Research workspace composition. Move existing controls to preserve command contracts.
(() => {
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
  shell.setAttribute('aria-label', 'UT27 실험 워크스페이스');
  document.querySelector('.app-header').after(shell);

  // The sidebar describes work and visible views; it is not a second command surface.
  const sidebar = node('aside', 'workspace-sidebar', `<div class="sidebar-heading">WORKSPACE</div><div class="workspace-project">${icon('layers')}<div><strong>UT27 Research</strong><small>수중 로봇 시뮬레이션</small></div></div><div class="sidebar-heading">작업</div>`);
  sidebar.append(nav);
  const navIcons = ['activity', 'folder', 'sliders'];
  [...nav.children].forEach((button, index) => button.insertAdjacentHTML('afterbegin', icon(navIcons[index])));
  const views = node('div', 'workspace-views', `<div class="sidebar-heading">관찰 화면</div><button type="button" data-camera-layout="split" aria-pressed="true">${icon('layers')}전체 보기</button><button type="button" data-camera-layout="front" aria-pressed="false">${icon('video')}전방 카메라</button><button type="button" data-camera-layout="hand" aria-pressed="false">${icon('video')}손 카메라</button><div class="sidebar-heading">실험 도구</div><button type="button" id="workspacePhysics">${icon('sliders')}물리 프로필</button><button type="button" id="workspaceCourse">${icon('crosshair')}시작 배치</button>`);
  sidebar.append(views, node('div', 'sidebar-foot', `${icon('cpu')}<span>MuJoCo · ROS 2<small>UT27 / local workspace</small></span>`));

  const stage = node('section', 'workspace-stage');
  const stageHeader = node('div', 'workspace-toolbar', `<div><span class="workspace-breadcrumb">UT27 / </span><strong id="workspaceTitle">실시간 관찰</strong></div><div class="workspace-toolbar-actions"><span class="workspace-hint">화면</span><button id="workspaceInspectorToggle" type="button" aria-pressed="true" aria-label="상세 패널 표시 전환">${icon('sidebar')}상세 패널</button></div>`);
  const pages = node('div', 'workspace-pages');
  pages.append(main, data, setup);
  stage.append(stageHeader, pages);
  shell.append(sidebar, stage);

  // Keep the original camera element intact so pop-out and collection can reuse it.
  const cameraLegend = camera.querySelector('legend');
  cameraLegend.textContent = '카메라';
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
    ['stereoLeftView', '전방 카메라 대기 중', '시뮬레이션을 시작하면 영상이 표시됩니다.'],
    ['stereoRightView', '손 카메라 대기 중', '포크와 목표물의 접촉을 확인합니다.'],
  ]) {
    byId(id).append(node('div', 'camera-empty', `${icon('video')}<strong>${title}</strong><small>${detail}</small>`));
  }

  const inspector = node('aside', 'workspace-inspector', '<div class="inspector-title"><strong>상세 패널</strong><button type="button" id="workspaceInspectorClose" aria-label="상세 패널 닫기">닫기</button></div><div class="inspector-tabs" role="tablist" aria-label="상세 패널 선택"><button type="button" id="inspectorControlTab" role="tab" aria-controls="inspectorControls" aria-selected="true">조종</button><button type="button" id="inspectorStatusTab" role="tab" aria-controls="telemetryPanel" aria-selected="false">기체 상태</button></div>');
  const inspectorBody = node('div', 'inspector-body');
  controls.id = 'inspectorControls';
  controls.setAttribute('role', 'tabpanel');
  controls.setAttribute('aria-labelledby', 'inspectorControlTab');
  telemetry.setAttribute('role', 'tabpanel');
  telemetry.setAttribute('aria-labelledby', 'inspectorStatusTab');
  telemetry.hidden = true;
  controls.querySelector('h2').remove();
  // Joysticks are available on demand; the control-enable state remains explicit.
  const pilot = byId('pilotControlGroup');
  const pilotDetails = node('details', 'pilot-disclosure', '<summary>수동 조종기</summary>');
  pilot.before(pilotDetails);
  pilotDetails.append(pilot);
  inspectorBody.append(controls, telemetry);
  inspector.append(inspectorBody);
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
  dock.setAttribute('aria-label', '조종 명령 로그 재생');
  replay.querySelector('legend').textContent = '조종 명령 재생';
  byId('rcReplaySlider').setAttribute('aria-label', '조종 명령 재생 위치');
  byId('rcReplayRate').setAttribute('aria-label', '조종 명령 재생 배속');
  byId('rcReplayRate').closest('label').firstChild.textContent = '배속 ';
  const replayNote = node('p', 'replay-note', '기록된 조종 명령을 시뮬레이터에 입력합니다. 영상 탐색과 구분해 사용하세요.');
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
  document.querySelector('.title-stack').innerHTML = '<h1>UT27 <span>Research Studio</span></h1>';
  byId('settingsOpen').textContent = '설정';
  byId('settingsDialog').querySelector('h2').textContent = '설정';
  byId('setup-heading').textContent = '실험 조건';
  byId('telemetryToggle').hidden = true;
  byId('cameraDialog').querySelector('h2').textContent = '카메라 확대';
  byId('recorderDialog').querySelector('h2').textContent = '시연 수집';
  byId('vehicleDetailsToggle').textContent = '상세';
  byId('controlDetailsToggle').textContent = '축 값';

  // Static, locally vendored Feather icons need no client-side library or network.
  const actionIcons = {
    stackStartBtn:'play', stackResetBtn:'square', settingsOpen:'settings', quickCenterBtn:'crosshair',
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
