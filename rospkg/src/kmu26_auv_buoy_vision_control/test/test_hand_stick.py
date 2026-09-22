import sys
from pathlib import Path
import unittest
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from hand_stick import detect_hand_sticks

class ShaftObservation(unittest.TestCase):
    def image(self):
        im=np.zeros((360,640,3),np.uint8);im[:]=(100,50,20)
        cv2.ellipse(im,(200,110),(28,45),0,0,360,(0,230,230),-1)
        return im

    def test_rod_with_yellow_anchor(self):
        im=self.image();cv2.line(im,(200,161),(245,250),(170,170,170),9)
        self.assertEqual(len(detect_hand_sticks(im)),1)

    def test_unrelated_gray_hull_rejected(self):
        im=self.image();cv2.rectangle(im,(500,0),(620,350),(140,140,140),-1)
        self.assertEqual(detect_hand_sticks(im),[])

    def test_no_yellow_anchor_rejected(self):
        im=np.zeros((360,640,3),np.uint8);cv2.line(im,(200,161),(245,250),(170,170,170),9)
        self.assertEqual(detect_hand_sticks(im),[])

    def test_actual_new_camera_missed_by_yolo(self):
        path=Path(__file__).resolve().parents[4]/'docs/assets/hand-camera-reference-20260920/actual-approach.jpg'
        detected=detect_hand_sticks(cv2.imread(str(path)))
        self.assertEqual(len(detected),1)
        self.assertTrue(180 < detected[0][2] < 315)
        self.assertTrue(190 < detected[0][3] < 300)

if __name__=='__main__':unittest.main()
