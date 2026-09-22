import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from yellow_ellipse import detect_yellow_ellipses


class YellowEllipse(unittest.TestCase):
    def test_small_visible_buoy_is_retained(self):
        image = np.zeros((360,640,3),np.uint8)
        cv2.ellipse(image,(330,220),(4,8),0,0,360,(0,220,220),-1)
        self.assertEqual(len(detect_yellow_ellipses(image)),1)

    def test_clipped_target_is_not_rejected_by_border(self):
        image = np.zeros((360,640,3),np.uint8)
        cv2.ellipse(image,(1,180),(15,30),0,0,360,(0,220,220),-1)
        self.assertEqual(len(detect_yellow_ellipses(image)),1)

    def test_blue_pool_is_not_a_yellow_target(self):
        image = np.full((360,640,3),(180,100,20),np.uint8)
        self.assertEqual(detect_yellow_ellipses(image),[])

    def test_multiple_candidates_are_not_reduced_to_largest(self):
        image = np.zeros((360,640,3),np.uint8)
        for x,axes in [(200,(8,16)),(400,(16,32))]:
            cv2.ellipse(image,(x,180),axes,0,0,360,(0,220,220),-1)
        self.assertEqual(len(detect_yellow_ellipses(image)),2)
