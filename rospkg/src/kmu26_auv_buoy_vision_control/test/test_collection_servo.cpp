#include "auv_buoy_vision_control/collection_servo.hpp"
#include <cassert>
#include <cmath>
using namespace auv_buoy_vision_control;
int main() {
  assert(compensate_rc_deadzone(0,40)==0);
  assert(compensate_rc_deadzone(10,40)>40);
  assert(compensate_rc_deadzone(-10,30)<-30);
  assert(compensate_rc_deadzone(200,40)==200);
  assert(compensate_rc_deadzone(-200,30)==-200);
  int direction=1;
  double target=0.5;
  for(int i=0;i<100;++i) target=search_depth_target(target,0.05,0.1,0.3,2.0,direction);
  assert(target>0.99 && target<1.01);
  target=2.0;
  assert(search_depth_target(target,0.05,0.1,0.3,2.0,direction)<2.0);
  assert(direction==-1);
  assert(search_depth_target(0.3,0.05,0.1,0.3,2.0,direction)>0.3);
  assert(visual_depth_target(1.0,0.5,0.7,0.3,2.0)>1.0);
  assert(visual_depth_target(1.0,-0.5,0.7,0.3,2.0)<1.0);
  assert(visual_depth_target(1.4,0.0,0.7,0.3,2.0)==1.4);
  assert(aligned_forward_pwm(0.5,0.0,1500,1580,0.4)==1500);
  assert(aligned_forward_pwm(0.0,0.5,1500,1580,0.4)==1500);
  assert(aligned_forward_pwm(0.0,0.0,1500,1580,0.4)==1580);
}
