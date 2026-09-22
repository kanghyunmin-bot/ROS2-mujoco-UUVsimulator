"""Simulation-only white shaft observations anchored to a visible yellow buoy.

Shape score is not a learned probability. These observations never label success.
"""
import cv2
import numpy as np


def detect_hand_sticks(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, (18, 60, 40), (45, 255, 255))
    ys, xs = np.nonzero(yellow)
    if len(xs) < 8:
        return []
    # The collar/occluding tine can separate the visible shaft from the float.
    max_gap = max(12., .35 * float(ys.max()-ys.min()+1))
    distance = cv2.distanceTransform(255-yellow, cv2.DIST_L2, 3)
    white = cv2.inRange(hsv, (0, 0, 45), (179, 65, 255))
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    candidates = []
    for contour in contours:
        if len(contour) < 5 or cv2.contourArea(contour) < 8:
            continue
        points = contour.reshape(-1, 2)
        eigenvalues = np.linalg.eigvalsh(np.cov(points.T))
        elongation = np.sqrt(eigenvalues[-1] / max(eigenvalues[0], 1.))
        gap = float(distance[points[:, 1], points[:, 0]].min())
        if elongation < 2.0 or gap > max_gap:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        score = min(.95, .65 + .05 * elongation)
        candidates.append((gap, (1, score, x+w/2, y+h/2, float(w), float(h), x, y, x+w, y+h)))
    # Association to the yellow float avoids selecting unconnected gray hulls.
    return [min(candidates, key=lambda item: item[0])[1]] if candidates else []
