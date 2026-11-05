#include "../control/drive.hpp"
#include <cassert>
#include <cmath>
using namespace robotracers;
int main() {
  Drive drive;
  assert(std::abs(drive.torque(255,0)-drive.p.stall_torque)<1e-10);
  assert(std::abs(drive.torque(-255,0)+drive.p.stall_torque)<1e-10);
  assert(drive.torque(0,30)<0 && drive.torque(0,-30)>0);
  for(double speed=-800;speed<=800;speed+=5)
    for(double pwm=-255;pwm<=255;pwm+=51) {
      assert(std::abs(drive.current(pwm,speed))<=drive.p.max_current);
      assert(std::abs(drive.torque(pwm,speed))<=drive.p.stall_torque);
    }
  drive.observe(0,2*pi/drive.p.counts_per_turn,.01);
  drive.observe(1,-2*pi/drive.p.counts_per_turn,.01);
  assert(drive.wheels[0].count==1 && drive.wheels[1].count==-1);
  drive.running=true;drive.command_w=1;drive.update(.01);
  assert(drive.wheels[0].pwm<0 && drive.wheels[1].pwm>0);
  drive.stop();
  assert(!drive.running && drive.wheels[0].pwm==0 && drive.wheels[1].pwm==0);
  drive.running=true;drive.command_v=.3;
  for(int i=1;i<1000;i++) {
    const double t=i*.01;
    const double angle=.3*t/drive.p.radius;
    drive.observe(0,angle,t);drive.observe(1,angle,t);drive.update(.01);
  }
  assert(std::abs(drive.wheels[0].velocity-.3)<.06);
  drive.update(.2);assert(!drive.running);
}
