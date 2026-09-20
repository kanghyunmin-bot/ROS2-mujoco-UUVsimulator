"""Offline visual sampling cues, never physical contact or success labels."""
import math
import cv2
import numpy as np


def yellow_cue(bgr):
    """Measure the largest yellow component in the recorded single-buoy scene.

    Area is the visible image fraction, not metric range. Edge clipping is
    reported as metadata and does not invalidate an observed yellow component.
    HSV thresholds are experiment heuristics, not a trained semantic detector.
    """
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([15, 90, 55]), np.array([42, 255, 255]))
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return dict(valid=False, area=0., cx=0., cy=0., clipped=False)
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x,y,bw,bh,area = map(int, stats[i]);cx,cy = centers[i]
    clipped = x <= 1 or y <= 1 or x+bw >= w-1 or y+bh >= h-1
    return dict(valid=area >= 12, area=area/(w*h), cx=float(cx/w), cy=float(cy/h), clipped=clipped)


def importance(ego, work, reviewed_proximity, strength):
    """Return the maximum applicable weight, without compounding overlapping cues."""
    if strength not in (1,2,3):
        raise ValueError('Unknown strength')
    values={1:(1.,1.,1.),2:(1.15,1.4,2.),3:(1.3,1.8,3.)}[strength]
    center=ego['valid'] and ego['area'] >= .0002 and math.hypot((ego['cx']-.5)/.22,(ego['cy']-.5)/.25) <= 1
    near=ego['valid'] and ego['area'] >= .003
    interaction=reviewed_proximity and work['valid'] and work['area'] >= .001
    cues={'image_center':bool(center),'large_ego_buoy':bool(near),'reviewed_work_proximity':bool(interaction),'confirmed_engagement':False}
    weight=max([1.]+[v for v,active in zip(values,[center,near,interaction]) if active])
    return weight,cues
