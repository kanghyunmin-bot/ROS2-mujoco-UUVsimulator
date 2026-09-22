import unittest
from reward import ForkReward


def observation(**changes):
    return dict(time_s=0., target_id='yellow', detached=False, eq_active=True,
                height_error_m=.2, fork_stem_distance_m=.5, camera_valid=True,
                visible=True, depth_m=1., yaw_rad=0.,
                release_right_fork_region=False, release_time_s=-1., **changes)


class Rewards(unittest.TestCase):
    def episode(self):
        r=ForkReward();o=observation();r.reset(o);return r,o

    def test_right_fork_required(self):
        for contact in (False,True):
            r,o=self.episode();o.update(time_s=10.,detached=True,eq_active=False,
                release_time_s=10.,release_right_fork_region=contact)
            self.assertEqual(r.step(o)['success'],contact)

    def test_release_deadline_uses_event_time(self):
        r,o=self.episode();o.update(time_s=35.1,detached=True,eq_active=False,
            release_time_s=34.9,release_right_fork_region=True)
        self.assertTrue(r.step(o)['success'])

    def test_fast_success_higher(self):
        values=[]
        for time in (10.,30.,36.):
            r,o=self.episode();o.update(time_s=time,detached=True,eq_active=False,
                release_time_s=time,release_right_fork_region=True)
            values.append(r.step(o)['reward'])
        self.assertGreater(values[0],values[1]);self.assertGreater(values[1],values[2])

    def test_no_spin_or_visibility_farming(self):
        r,o=self.episode();o.update(time_s=1.,visible=False);r.step(o)
        o.update(time_s=2.,yaw_rad=1.);self.assertLessEqual(r.step(o)['reward'],0)
        o.update(time_s=3.,visible=True);self.assertIn('rediscovery',r.step(o)['terms'])
        o.update(time_s=4.,visible=False);r.step(o)
        o.update(time_s=5.,visible=True,yaw_rad=2.)
        self.assertNotIn('rediscovery',r.step(o)['terms'])

    def test_contact_paid_once_only_before_release(self):
        r,o=self.episode();o.update(time_s=1.,right_fork_stem_contact_active=True)
        self.assertEqual(r.step(o)['terms']['right_fork_contact'],2.)
        o.update(time_s=2.)
        self.assertNotIn('right_fork_contact',r.step(o)['terms'])
        r,o=self.episode();o.update(time_s=1.,detached=True,eq_active=False,
            release_time_s=1.,right_fork_stem_contact_active=True)
        self.assertNotIn('right_fork_contact',r.step(o)['terms'])

    def test_approach_cannot_be_farmed(self):
        r,o=self.episode();o.update(time_s=1.,fork_stem_distance_m=.1)
        self.assertGreater(r.step(o)['terms']['approach'],0.)
        o.update(time_s=2.,fork_stem_distance_m=.5);r.step(o)
        o.update(time_s=3.,fork_stem_distance_m=.1)
        self.assertEqual(r.step(o)['terms']['approach'],0.)

    def test_release_tiers_and_stale_hand(self):
        scores=[]
        for hand,contact,delay in [(False,False,.1),(True,False,.1),(True,True,.1),(True,False,1.)]:
            r,o=self.episode();o.update(time_s=2.,hand_visible=hand);r.step(o)
            o.update(time_s=2.+delay,detached=True,eq_active=False,release_time_s=2.+delay,release_right_fork_region=contact)
            result=r.step(o);scores.append(result['terms']['terminal'])
            self.assertTrue(result['released'])
        self.assertEqual(scores,[3.,8.,20.,3.])

    def test_release_after_deadline_no_tier_reward(self):
        r,o=self.episode();o.update(time_s=36.,detached=True,eq_active=False,release_time_s=36.,release_right_fork_region=True)
        result=r.step(o)
        self.assertFalse(result['released'])
        self.assertEqual(result['release_tier'],'invalid')
        self.assertLess(result['terms']['terminal'],0)

    def test_alignment_only_new_best(self):
        r,o=self.episode();o.update(time_s=1.,height_error_m=0.)
        self.assertGreater(r.step(o)['terms']['height_alignment'],0)
        o.update(time_s=2.,height_error_m=.2);r.step(o)
        o.update(time_s=3.,height_error_m=0.)
        self.assertEqual(r.step(o)['terms']['height_alignment'],0)

if __name__=='__main__':unittest.main()
