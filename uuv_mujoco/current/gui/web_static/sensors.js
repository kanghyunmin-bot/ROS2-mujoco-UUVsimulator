// Display-only telemetry. Missing readings must never become numerical zero.
(() => {
  const number = value => value === null || value === undefined || value === '' ? NaN : Number(value);
  const fixed = (value, digits = 2) => Number.isFinite(number(value)) ? number(value).toFixed(digits) : '—';
  const age = value => Number.isFinite(number(value)) ? `${fixed(value, 1)} s 전 수신` : '미수신';
  const set = (id, value) => { const el = document.getElementById(id); if (el && el.textContent !== value) el.textContent = value; };
  window.renderStationSensors = payload => {
    const t = payload.telemetry || {}, camera = payload.stereo_camera || {};
    const hasImu = Number.isFinite(number(t.imu_age_s));
    set('sensorAttitude', hasImu ? `${fixed(t.roll_deg)} / ${fixed(t.pitch_deg)} / ${fixed(t.yaw_deg)} °` : '— / — / —');
    set('sensorImuAge', `ROLL / PITCH / YAW · ${age(t.imu_age_s)}`);
    set('sensorDepth', fixed(t.depth_m, 3));
    set('sensorDepthAge', `${t.depth_source || 'unavailable'} · ${age(t.depth_age_s)}`);
    set('sensorVelocity', (t.velocity_xyz || []).length === 3 ? t.velocity_xyz.map(v => fixed(v, 3)).join(' / ') : '— / — / —');
    set('sensorVelocitySource', `${t.velocity_source || 'unavailable'} · XYZ (원본 좌표계)`);
    set('sensorCameras', `${fixed(camera.left?.age_s, 1)} / ${fixed(camera.right?.age_s, 1)} s`);
  };
})();
