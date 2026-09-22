import sys
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gui.recording_partition import recording_partition

class PartitionTest(unittest.TestCase):
    def test_camera_geometry_and_resolution_isolate_data(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);scene=root/'scene.xml'
            template='<mujoco><worldbody><camera name="stereo_left" pos="0 0 0" fovy="70"/><camera name="stereo_right" pos="0 0 0" fovy="{}"/></worldbody></mujoco>'
            scene.write_text(template.format(82))
            old,_=recording_partition(root,scene,{'width':640,'height':360})
            scene.write_text(template.format(35))
            new,profile=recording_partition(root,scene,{'width':640,'height':360})
            again,_=recording_partition(root,scene,{'width':640,'height':360})
            high,_=recording_partition(root,scene,{'width':1280,'height':720})
            self.assertNotEqual(old,new);self.assertNotEqual(new,high);self.assertEqual(new,again)
            self.assertIn('vla-experiments',str(new));self.assertEqual(profile['cameras']['stereo_right']['fovy'],[35.])
            self.assertFalse(new.exists())
            stable, stable_profile = recording_partition(
                root, scene, {'width': 640, 'height': 360},
                control_profile='yaw-stable-20260920-v1',
            )
            self.assertNotEqual(new, stable)
            self.assertEqual(stable_profile['control_profile'], 'yaw-stable-20260920-v1')

    def test_missing_camera_fails_without_mixing_into_legacy(self):
        with TemporaryDirectory() as directory:
            root=Path(directory);scene=root/'scene.xml';scene.write_text('<mujoco/>')
            with self.assertRaises(ValueError):recording_partition(root,scene,{})

if __name__=='__main__':unittest.main()
