import unittest
from transition import attach_feedback
from evaluation import summary,promote,assessment
class Tests(unittest.TestCase):
 def test_wait_reward_attaches_previous_not_pending(self):
  t=[{},{}];attach_feedback(t,{'feedback':{'action_id':0,'reward':20},'done':True,'last_applied_id':0})
  self.assertEqual(len(t),1);self.assertEqual(t[0]['feedback']['reward'],20)
 def test_delayed_feedback_id(self):
  t=[{},{},{}];attach_feedback(t,{'feedback':{'action_id':0,'reward':1}})
  self.assertIn('feedback',t[0]);self.assertNotIn('feedback',t[2])
 def test_reject_duplicate(self):
  t=[{}];d={'feedback':{'action_id':0}};attach_feedback(t,d)
  with self.assertRaises(ValueError):attach_feedback(t,d)
 def test_reject_worse_even_higher_reward(self):
  b={'n':6,'fork':1,'hand_or_fork':2,'released':4,'mean_reward':2}
  self.assertFalse(promote(b,{**b,'released':3,'mean_reward':10}))
  self.assertTrue(promote(b,{**b,'released':5}))
 def test_reject_timing_noise(self):
  b={'n':6,'fork':0,'hand_or_fork':0,'released':1,'mean_reward':-.56}
  self.assertFalse(promote(b,{**b,'mean_reward':-.43}))
 def test_reject_unchanged_policy(self):
  b={'n':6,'fork':0,'hand_or_fork':0,'released':1,'mean_reward':-.56}
  self.assertFalse(promote(b,{**b,'released':6,'mean_reward':10},parameter_changed=False))
 def test_missing_environment_neither_promotes_nor_rolls_back(self):
  b={'n':6,'fork':0,'hand_or_fork':0,'released':1,'mean_reward':0}
  for reward in (-10,10):
   c={**b,'n':5,'mean_reward':reward}
   self.assertFalse(promote(b,c))
   self.assertEqual(assessment(b,c),'incomplete_evaluation')
 def test_inconclusive_keeps_learning_without_promoting(self):
  b={'n':6,'fork':0,'hand_or_fork':0,'released':1,'mean_reward':.1749342591440123}
  c={**b,'mean_reward':.3679900184069616}
  self.assertFalse(promote(b,c))
  self.assertEqual(assessment(b,c),'continue_training')
  self.assertEqual(assessment(b,{**c,'released':0}),'rollback')
  self.assertEqual(assessment(b,{**c,'mean_reward':-1}),'rollback')
  self.assertEqual(assessment(b,{**c,'released':2}),'promote')
if __name__=='__main__':unittest.main()
