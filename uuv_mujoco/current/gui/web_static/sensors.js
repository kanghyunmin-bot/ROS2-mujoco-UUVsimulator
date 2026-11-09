// Display-only telemetry. Missing readings must never become numerical zero.
(() => {
  const translate = (english, korean) => window.StationLanguage?.text(english, korean) ?? english;
  const number = value => value === null || value === undefined || value === '' ? NaN : Number(value);
  const fixed = (value, digits = 2) => Number.isFinite(number(value)) ? number(value).toFixed(digits) : '—';
  const age = value => Number.isFinite(number(value)) ? translate(`Received ${fixed(value, 1)} s ago`, `${fixed(value, 1)} s 전 수신`) : translate('Not received', '미수신');
  const set = (id, value) => { const el = document.getElementById(id); if (el && el.textContent !== value) el.textContent = value; };
  window.renderStationSensors = payload => {
    const t = payload.telemetry || {}, camera = payload.stereo_camera || {};
    const blue = document.body?.dataset?.vehicle === 'bluerov2_standard';
    const hasImu = Number.isFinite(number(t.imu_age_s));
    set('sensorAttitude', hasImu ? `${fixed(t.roll_deg)} / ${fixed(t.pitch_deg)} / ${fixed(t.yaw_deg)} °` : '— / — / —');
    set('sensorImuAge', `ROLL / PITCH / YAW · ${age(t.imu_age_s)}`);
    set('sensorDepth', fixed(t.depth_m, 3));
    set('sensorDepthAge', `${t.depth_source || 'unavailable'} · ${age(t.depth_age_s)}`);
    set('sensorVelocity', (t.velocity_xyz || []).length === 3 ? t.velocity_xyz.map(v => fixed(v, 3)).join(' / ') : '— / — / —');
    set('sensorVelocitySource', `${t.velocity_source || 'unavailable'} · ${t.velocity_frame || translate('source frame', '원본 좌표계')} · ${age(t.velocity_age_s)}`);
    set('sensorCameras', blue ? `${fixed(camera.left?.age_s, 1)} s`
      : `${fixed(camera.left?.age_s, 1)} / ${fixed(camera.right?.age_s, 1)} s`);
    set('sensorCameraDescription', blue ? translate('Front · time since received', '전방 · 수신 후 경과 시간') : translate('Front / hand · time since received', '전방 / 손 · 수신 후 경과 시간'));
    const simTime = number(t.sitl_mavlink_status?.sim_time_s);
    const clockFresh = number(t.sitl_mavlink_status_age_s) < 2.5;
    const captureAge = side => {
      const stamp = number(camera[side]?.capture_stamp_s);
      const delta = simTime - stamp;
      return clockFresh && Number.isFinite(stamp) && delta >= 0 ? delta : NaN;
    };
    const leftAge = captureAge('left'), rightAge = captureAge('right');
    set('sensorCameraCaptureAge', blue ? translate(`Capture delay · ${fixed(leftAge, 2)} s (simulation time)`, `촬영 지연 · ${fixed(leftAge, 2)} s (시뮬 시간)`)
      : translate(`Capture delay · ${fixed(leftAge, 2)} / ${fixed(rightAge, 2)} s (simulation time)`, `촬영 지연 · ${fixed(leftAge, 2)} / ${fixed(rightAge, 2)} s (시뮬 시간)`));
    const captureNote = document.getElementById('sensorCameraCaptureAge');
    if (captureNote) captureNote.style.color = (blue ? [leftAge] : [leftAge, rightAge])
      .some(value => value > .25) ? '#ffbd69' : '';
  };
})();
