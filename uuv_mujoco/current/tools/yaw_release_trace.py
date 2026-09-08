"""Read-only MAVLink log extraction for reproducible yaw plant experiments."""
from pathlib import Path
import hashlib
import numpy as np


def validate_trace(trace):
    if not np.isfinite(trace['initial_yaw_rate_flu_radps']):
        raise ValueError('initial yaw rate must be finite')
    rows = trace['rows']
    if len(rows) < 2:
        raise ValueError('trace needs at least two samples')
    times = np.array([r['t_s'] for r in rows])
    if not np.all(np.isfinite(times)) or times[0] != 0 or np.any(np.diff(times) <= 0):
        raise ValueError('trace timestamps must increase strictly from zero')
    for row in rows:
        if not np.isfinite(row['observed_yaw_rate_flu_radps']) or not 800 <= row['rc4_pwm'] <= 2200:
            raise ValueError('invalid recorded attitude or RC input')
        pwm = np.asarray(row['servo_pwm'])
        if pwm.shape != (8,) or not np.all(np.isfinite(pwm)) or np.any(pwm < 1100) or np.any(pwm > 1900):
            raise ValueError('trace needs eight final servo PWM values in 1100..1900')
    if not 0 < trace['release_s'] < times[-1]:
        raise ValueError('release must be inside trace')
    return trace


def extract_tlog(path, *, release_index=-1, before_s=2., after_s=4.):
    from pymavlink import mavutil
    path = Path(path)
    link = mavutil.mavlink_connection(str(path))
    streams = {k: [] for k in ('RC_CHANNELS', 'SERVO_OUTPUT_RAW', 'ATTITUDE')}
    while True:
        message = link.recv_match()
        if message is None:
            break
        kind = message.get_type()
        if kind not in streams or (kind == 'SERVO_OUTPUT_RAW' and message.port != 0):
            continue
        # Shared autopilot boot clock, not potentially delayed receive timestamps.
        t = message.time_usec / 1e6 if kind == 'SERVO_OUTPUT_RAW' else message.time_boot_ms / 1000.
        streams[kind].append((t, message.to_dict()))
    link.close()
    for stream in streams.values():
        if not stream or any(b[0] < a[0] for a,b in zip(stream,stream[1:])):
            raise ValueError('missing stream or boot-clock reset; split the log first')
    rc = streams['RC_CHANNELS']
    releases = [t for (previous_t, previous), (t, current) in zip(rc,rc[1:])
                if abs(previous['chan4_raw']-1500)>20 and abs(current['chan4_raw']-1500)<=20
                and t-before_s >= max(s[0][0] for s in streams.values())
                and t+after_s <= min(s[-1][0] for s in streams.values())]
    if not releases:
        raise ValueError('no yaw release with complete surrounding telemetry')
    release = releases[release_index]; start = release-before_s; end = release+after_s
    def latest(kind,t):
        stream = streams[kind]
        i = np.searchsorted([s[0] for s in stream],t,side='right')-1
        return stream[i]
    times = sorted(set([start,end,release]+[t for stream in streams.values() for t,_ in stream if start<t<end]))
    rows=[]
    for t in times:
        st, servo = latest('SERVO_OUTPUT_RAW',t)
        rt, control = latest('RC_CHANNELS',t)
        at, attitude = latest('ATTITUDE',t)
        rows.append(dict(t_s=t-start, rc4_pwm=control['chan4_raw'],
            servo_pwm=[servo[f'servo{i}_raw'] for i in range(1,9)],
            observed_yaw_rate_flu_radps=-attitude['yawspeed'],
            servo_age_s=t-st, rc_age_s=t-rt, attitude_age_s=t-at))
    maximum_age = max(max(r[k] for k in ('servo_age_s','rc_age_s','attitude_age_s')) for r in rows)
    if maximum_age > .5:
        raise ValueError(f'telemetry stale by {maximum_age:.3f}s; refusing silent interpolation')
    return validate_trace(dict(source='recorded_final_SERVO_OUTPUT_RAW', source_path=str(path.resolve()),
        sha256=hashlib.sha256(path.read_bytes()).hexdigest(), release_boot_s=release,
        release_s=before_s, rows=rows, initial_yaw_rate_flu_radps=rows[0]['observed_yaw_rate_flu_radps'],
        available_release_boot_s=releases, max_telemetry_age_s=maximum_age,
        assumptions=['level initial attitude; depth 1.5m; zero translation/current',
                     'FRD recorded yawspeed converted to FLU by negation',
                     'zero-order hold final PWM; RC is annotation, not controller input',
                     'initial thruster lag states zero; initial rate from telemetry']))
