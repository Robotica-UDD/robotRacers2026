#pragma once
// Motor and encoder control independent of Webots and Arduino. SI units.
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robotracers {
constexpr double pi = 3.14159265358979323846;
inline double limit(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
struct Parameters {
  double radius = .0115, track = .19, counts_per_turn = 59.94;
  double voltage = 6., no_load_rpm = 6500., no_load_current = .17;
  double stall_current = 1.5, stall_torque = .008825985;
  double max_current = 1.53, acceleration = 1.0;
};
struct Wheel {
  std::int64_t count = 0, previous_count = 0, start_count = 0;
  double event_time = 0, event_velocity = 0, velocity = 0;
  double reference = 0, integral = 0, startup_pwm = 35, pwm = 0;
  int direction = 0;
  bool started = false;
};
class Drive {
 public:
  explicit Drive(Parameters parameters = {}) : p(parameters) {}
  Parameters p;
  Wheel wheels[2];
  double command_v = 0, command_w = 0;
  bool running = false;

  void observe(int i, double angle, double now) {
    Wheel &s = wheels[i];
    s.count = std::llround(angle * p.counts_per_turn / (2*pi));
    const auto delta = s.count - s.previous_count;
    const double distance = 2*pi*p.radius/p.counts_per_turn;
    if (delta) {
      const double dt = std::max(.000001, now-s.event_time);
      const double sample = delta*distance/dt;
      const double alpha = dt/(.024+dt);
      s.event_velocity += alpha*(sample-s.event_velocity);
      s.previous_count = s.count; s.event_time = now;
    }
    const double age = now-s.event_time;
    s.velocity = std::copysign(std::min(std::abs(s.event_velocity), distance/std::max(.000001, age)), s.event_velocity);
  }

  void stop() {
    running = false; command_v = command_w = 0;
    for (auto &s : wheels) {
      s.reference = s.integral = s.pwm = 0;
      s.direction = 0; s.started = false; s.startup_pwm = 35;
    }
  }

  void update(double dt) {
    if (!running || dt<=0 || dt>.1) { stop(); return; }
    const double targets[] = {command_v-command_w*p.track/2, command_v+command_w*p.track/2};
    for (int i=0;i<2;i++) {
      auto &s = wheels[i];
      const double target = limit(targets[i], -1.6, 1.6);
      if (std::abs(target)<.0005) {
        s.reference = s.integral = s.pwm = 0; s.direction = 0; s.started = false;
        continue;
      }
      const int wanted_direction = target>0 ? 1 : -1;
      if (s.direction != wanted_direction) {
        s.integral = 0; s.start_count = s.count; s.started = false;
        s.startup_pwm = 35; s.direction = wanted_direction;
      }
      s.reference += limit(target-s.reference, -p.acceleration*dt, p.acceleration*dt);
      const double desired = std::max(0., wanted_direction*s.reference)*1000;
      const double measured = wanted_direction*s.velocity*1000;
      const double error = desired-measured;
      // Initial values come from the physically tested REV3.10 velocity loop.
      const double feedforward = 35+.125*desired;
      if (!s.started) {
        if (wanted_direction*(s.count-s.start_count)>=1) {
          s.started = true;
          s.integral = limit(.4*(s.startup_pwm-feedforward-.10*error), -120, 125);
        } else {
          s.startup_pwm = std::min(160., s.startup_pwm+300*dt);
          s.pwm = wanted_direction*s.startup_pwm;
          continue;
        }
      }
      const double free = feedforward+.10*error+s.integral;
      if ((free>0 && free<160) || (free>=160 && error<0) || (free<=0 && error>0))
        s.integral = limit(s.integral+error*dt, -120, 125);
      s.pwm = wanted_direction*limit(feedforward+.10*error+s.integral, 0, 160);
    }
  }

  double current(double pwm, double angular_velocity) const {
    const double resistance = p.voltage/p.stall_current;
    const double ke = (p.voltage-p.no_load_current*resistance)/(p.no_load_rpm*2*pi/60);
    return limit((p.voltage*limit(pwm/255.,-1,1)-ke*angular_velocity)/resistance,
                 -p.max_current, p.max_current);
  }
  double torque(double pwm, double angular_velocity) const {
    const double kt = p.stall_torque/p.stall_current;
    // Smooth mechanical losses inferred from the no-load current. Provisional.
    const double loss = kt*p.no_load_current*std::tanh(angular_velocity/1.0);
    return limit(kt*current(pwm,angular_velocity)-loss, -p.stall_torque, p.stall_torque);
  }
};
} // namespace robotracers
