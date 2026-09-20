import numpy as np
import pytest
cv2=pytest.importorskip('cv2')
from kmu26_auv_vla_data_collector.visual_importance import yellow_cue,importance

def test_blank_rejected_and_clipped_kept():
 im=np.zeros((100,100,3),np.uint8);assert not yellow_cue(im)['valid']
 im[0:20,0:20]=(0,255,255);c=yellow_cue(im);assert c['clipped'] and c['valid']
 assert importance(c,c,True,3)[0]==3

def test_center_size_and_review_gate():
 im=np.zeros((100,100,3),np.uint8);im[45:55,45:55]=(0,255,255);c=yellow_cue(im)
 assert c['valid'] and c['area']==.01
 assert importance(c,c,False,2)[0]==1.4
 weight,cues=importance(c,c,True,2);assert weight==2 and not cues['confirmed_engagement']
 assert importance(c,c,True,1)[0]==1
 assert importance(c,c,True,3)[0]==3

def test_off_center_small_object_not_rewarded():
 c=dict(valid=True,area=.0003,cx=.1,cy=.1)
 assert importance(c,c,False,3)[0]==1
