"""Opt-in yellow ellipse observations for the simulation teacher, never success labels."""
import cv2
import numpy as np


def detect_yellow_ellipses(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (18, 60, 40), (45, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if len(contour) < 5 or area < 8:
            continue
        _, axes, _ = cv2.fitEllipse(contour)
        minor, major = sorted(axes)
        if minor <= 0 or not 1.2 <= major / minor <= 4.0:
            continue
        fit = area / max(1., np.pi * minor * major / 4.)
        if fit < .65:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        # This score measures silhouette fit, not a neural-network probability.
        detections.append((0, min(.95, fit), x+w/2, y+h/2, float(w), float(h), x, y, x+w, y+h))
    return detections
