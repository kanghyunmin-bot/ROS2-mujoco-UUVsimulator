"""Stateless RGB-only residual observations shared by training and deployment.

These color/shape cues are simulation-oriented visual measurements, not ground
truth pose, learned perception, metric range, or a certified fork calibration.
"""
import cv2
import numpy as np

STATE_KEYS = ('prev_command', 'dvl_velocity', 'angular_velocity',
              'linear_acceleration', 'attitude', 'depth', 'altitude', 'validity')
VISUAL_CONTRACT = 'rgb_target_cues_v1'
VISUAL_DIM = 26


def _largest(mask, minimum=8):
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= minimum]
    if not candidates:
        return None
    i = max(candidates, key=lambda j: stats[j, cv2.CC_STAT_AREA])
    return labels == i, stats[i], centers[i]


def _camera_features(value, target):
    image = np.asarray(value)
    if image.ndim == 4 and image.shape[0] == 1:
        image = image[0]
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8 or min(image.shape[:2]) < 8:
        raise ValueError('Visual residual requires one uint8 RGB image per camera')
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    h, w = image.shape[:2]
    yellow = _largest((hsv[..., 0] >= 18) & (hsv[..., 0] <= 45) &
                      (hsv[..., 1] >= 60) & (hsv[..., 2] >= 40))
    result = np.zeros(13, dtype=np.float32)
    if yellow is None:
        return result
    _, stats, center = yellow
    x, y, bw, bh, area = stats
    result[:7] = [1, center[0]/w-target[0], center[1]/h-target[1],
                  bw/w, bh/h, area/(h*w), int(x == 0 or y == 0 or x+bw == w or y+bh == h)]
    # Associate a bright shaft only near the observed yellow float. A missing
    # detection is explicit; never substitute simulator visibility or position.
    region = np.zeros((h, w), dtype=bool)
    region[max(0,y-bh):min(h,y+3*bh), max(0,x-bw):min(w,x+2*bw)] = True
    shaft = _largest(region & (hsv[..., 1] <= 65) & (hsv[..., 2] >= 100))
    if shaft is not None:
        mask, ss, sc = shaft
        yy, xx = np.nonzero(mask)
        covariance = np.cov(np.stack([xx, yy]))
        values, vectors = np.linalg.eigh(covariance)
        if values[1] >= 4*max(values[0], 1):
            direction = vectors[:, 1]
            if direction[1] < 0:
                direction = -direction
            angle = np.arctan2(direction[0], direction[1])/np.pi
            result[7:] = [1, sc[0]/w-target[0], sc[1]/h-target[1], ss[2]/w, ss[3]/h, angle]
    return result


def visual_features(observations):
    """Return 26 normalized image cues; no metric geometry is read."""
    try:
        return np.concatenate([_camera_features(observations['video.ego'], (.45, .5)),
                               _camera_features(observations['video.buoy_release'], (.48, .75))])
    except KeyError as error:
        raise ValueError('Both RGB camera observations are required') from error


def residual_state(observations, observation_contract='sensor_v1'):
    state = np.concatenate([np.asarray(observations['state.'+k]).reshape(-1) for k in STATE_KEYS]).astype(np.float32)
    if state.shape != (23,) or not np.isfinite(state).all():
        raise ValueError('Residual deployment needs finite 23-dimensional state')
    if observation_contract == 'sensor_v1':
        return state
    if observation_contract != VISUAL_CONTRACT:
        raise ValueError('Unsupported residual observation contract')
    return np.concatenate([state, visual_features(observations)])
