#include "../../firmware/RobotRacers_Prueba_WiFi/WheelVelocityControl.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

using robotracers::WheelVelocityControl;

namespace {
void check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "Línea " << line << ": " << expression << '\n';
    std::exit(1);
  }
}
#define CHECK(expression) check(bool(expression), #expression, __LINE__)

struct Rig {
  WheelVelocityControl control;
  double time = 0.;
  int64_t count = 0;
  Rig() { control.reset(count, time); }
  int step(int64_t next_count, double dt = .01) {
    time += dt;
    count = next_count;
    const int pwm = control.update(count, time);
    CHECK(std::abs(pwm) <= WheelVelocityControl::PWM_MAX);
    CHECK(std::isfinite(control.reference()));
    CHECK(std::isfinite(control.speed()));
    return pwm;
  }
};

void reset_and_idle() {
  Rig rig;
  CHECK(rig.control.target() == 0 && rig.control.pwm() == 0);
  CHECK(!rig.control.startup() && !rig.control.stalled());
  for (int i = 0; i < 1000; ++i) CHECK(rig.step(i / 10) == 0);
  CHECK(rig.control.reference() == 0.);
  CHECK(!rig.control.stalled());
  rig.control.setTarget(1000);
  CHECK(rig.control.target() == 100);
  rig.control.setTarget(-1000);
  CHECK(rig.control.target() == -100);
}

void startup_ramp() {
  Rig rig;
  rig.control.setTarget(100);
  CHECK(rig.step(0) == 35);
  int previous = 35;
  bool reached_limit = false;
  for (int i = 0; i < 480; ++i) {
    const int pwm = rig.step(0);
    CHECK(pwm >= previous && pwm <= 160);
    CHECK(pwm - previous <= 4);  // 300 PWM/s, con redondeo entero.
    CHECK(rig.control.startup() && !rig.control.stalled());
    CHECK(rig.control.reference() >= 0. && rig.control.reference() <= 100.);
    reached_limit |= pwm == 160;
    previous = pwm;
  }
  CHECK(reached_limit);
}

void two_counts_and_smooth_transfer() {
  Rig rig;
  rig.control.setTarget(33);
  for (int i = 0; i < 20; ++i) rig.step(0);
  rig.step(1);
  CHECK(rig.control.startup());
  for (int i = 0; i < 9; ++i) rig.step(1);
  const int previous = rig.control.pwm();
  const int transfer = rig.step(2);
  CHECK(!rig.control.startup());
  CHECK(transfer > 0);
  CHECK(std::abs(transfer - previous) <= 4);
  CHECK(!rig.control.stalled());
  // Una lectura posterior en el sentido contrario no invierte la salida.
  CHECK(rig.step(1) >= 0);
}

void wrong_encoder_direction_does_not_learn() {
  for (const int direction : {-1, 1}) {
    Rig rig;
    rig.control.setTarget(direction * 70);
    for (int i = 1; i <= 490; ++i) {
      const int pwm = rig.step(-direction * (i / 10));
      CHECK(pwm * direction > 0);
      CHECK(rig.control.startup() && !rig.control.stalled());
    }
    for (int i = 0; i < 30; ++i) rig.step(rig.count - direction);
    CHECK(rig.control.stalled() && rig.control.pwm() == 0);
  }
}

void zero_cancels_and_does_not_restart() {
  Rig rig;
  rig.control.setTarget(70);
  for (int i = 0; i < 25; ++i) rig.step(0);
  CHECK(rig.control.pwm() > 35);
  rig.control.setTarget(0);
  CHECK(rig.control.pwm() == 0 && rig.control.reference() == 0.);
  CHECK(!rig.control.startup());
  for (int i = 0; i < 1000; ++i) CHECK(rig.step(i) == 0);
  CHECK(rig.control.target() == 0 && !rig.control.stalled());
  for (int i = 0; i < 100; ++i) CHECK(rig.step(rig.count) == 0);
  rig.control.setTarget(-33);  // Solamente una consigna nueva permite arrancar.
  CHECK(rig.step(rig.count) == -35);
  CHECK(rig.control.startup());
}

void stall_is_latched() {
  Rig rig;
  rig.control.setTarget(100);
  for (int i = 0; i < 490; ++i) rig.step(0);
  CHECK(!rig.control.stalled());
  for (int i = 0; i < 30; ++i) rig.step(0);
  CHECK(rig.control.stalled() && rig.control.pwm() == 0);
  rig.control.setTarget(0);
  CHECK(rig.control.stalled());
  rig.control.setTarget(-70);
  for (int i = 0; i < 100; ++i) CHECK(rig.step(-i) == 0);
  CHECK(rig.control.stalled());
  rig.control.reset(rig.count, rig.time);
  CHECK(!rig.control.stalled() && rig.control.target() == 0);
  CHECK(rig.step(rig.count) == 0);
  rig.control.setTarget(-70);
  CHECK(rig.step(rig.count) == -35);
}

void no_repeated_startup_and_running_stall() {
  Rig rig;
  rig.control.setTarget(15);
  rig.step(0); rig.step(1); rig.step(2);
  CHECK(!rig.control.startup());
  for (int i = 0; i < 600; ++i) {
    rig.step(2 + i / 20);  // Movimiento lento y cuantizado, con 190 ms entre cuentas.
    CHECK(!rig.control.startup() && !rig.control.stalled());
  }
  const int64_t stopped_count = rig.count;
  for (int i = 0; i < 490; ++i) {
    rig.step(stopped_count);
    CHECK(!rig.control.startup());
  }
  for (int i = 0; i < 30; ++i) rig.step(stopped_count);
  CHECK(rig.control.stalled() && rig.control.pwm() == 0);
}

void reversal_and_target_flapping() {
  for (const bool flap : {false, true}) {
    Rig rig;
    rig.control.setTarget(70);
    for (int i = 1; i <= 60; ++i) rig.step(i / 5);
    CHECK(!rig.control.startup() && rig.control.reference() > 0.);
    rig.control.setTarget(-33);
    for (int i = 0; i < 5; ++i) rig.step(rig.count);
    if (flap) rig.control.setTarget(70);
    bool zero_seen = false, restarted = false;
    const int expected_direction = flap ? 1 : -1;
    const int before = rig.control.pwm();
    for (int i = 0; i < 300; ++i) {
      const int pwm = rig.step(rig.count);
      if (rig.control.reference() == 0. && pwm == 0) zero_seen = true;
      if (!zero_seen) CHECK(pwm >= 0 && pwm <= before);
      if (zero_seen && pwm != 0) {
        CHECK(pwm == expected_direction * 35);
        CHECK(rig.control.reference() * expected_direction > 0.);
        restarted = true;
        break;
      }
    }
    CHECK(zero_seen && restarted && !rig.control.stalled());
    const int64_t base = rig.count;
    rig.step(base + expected_direction);
    rig.step(base + 2 * expected_direction);
    CHECK(!rig.control.startup());
    CHECK(rig.control.pwm() * expected_direction >= 0);
  }
}

void zero_then_reverse_waits_for_rest() {
  Rig rig;
  rig.control.setTarget(70);
  for (int i = 1; i <= 100; ++i) rig.step(i / 2);
  CHECK(!rig.control.startup() && rig.control.speed() > 5.);
  const int64_t coast_count = rig.count;
  rig.control.setTarget(0);
  for (int i = 0; i < 5; ++i) CHECK(rig.step(coast_count) == 0);
  CHECK(rig.control.speed() > 5.);
  rig.control.setTarget(-33);  // El emisor puede pasar por cero antes de invertir.
  CHECK(rig.step(coast_count) == 0);
  bool restarted = false;
  for (int i = 0; i < 200; ++i) {
    const int pwm = rig.step(coast_count);
    CHECK(pwm <= 0);
    if (pwm < 0) {
      CHECK(pwm == -35);
      CHECK(std::abs(rig.control.speed()) <= 5.);
      CHECK(rig.control.startup());
      restarted = true;
      break;
    }
  }
  CHECK(restarted && !rig.control.stalled());
}

void pi_limits_and_antiwindup() {
  Rig rig;
  rig.control.setTarget(100);
  rig.step(0); rig.step(1); rig.step(2);
  int64_t count = 2;
  for (int i = 0; i < 500; ++i) {
    if (i % 20 == 0) ++count;
    CHECK(rig.step(count) >= 0);
  }
  CHECK(rig.control.pwm() >= 150);
  // La carga desaparece: el encoder indica sobrevelocidad. El integrador debe
  // salir de la saturación, sin invertir el motor ni mantener su máximo.
  for (int i = 0; i < 150; ++i) {
    count += 3;
    CHECK(rig.step(count) >= 0);
  }
  CHECK(rig.control.pwm() <= 20);
  rig.control.setTarget(20);
  bool recovered = false;
  for (int i = 0; i < 100; ++i) recovered |= rig.step(count) > 0;
  CHECK(recovered && !rig.control.stalled());
}

struct FrictionPlant {
  double position = 0., velocity = 0.;
  bool moving = false;
  int64_t counts() const { return int64_t(position / WheelVelocityControl::MM_PER_COUNT); }
  void step(int pwm, double dt) {
    if (!moving && std::abs(pwm) >= 90) moving = true;
    const double steady = moving && std::abs(pwm) > 45 ? std::copysign(3. * (std::abs(pwm) - 45), double(pwm)) : 0.;
    velocity += (steady - velocity) * (1. - std::exp(-dt / .10));
    position += velocity * dt;
    if (std::abs(velocity) < .2 && std::abs(pwm) < 45) moving = false;
  }
};

void quantized_friction_plant() {
  for (const int desired : {15, 33, 70, -15, -33, -70}) {
    Rig rig;
    FrictionPlant plant;
    rig.control.setTarget(desired);
    double mean_speed = 0., mean_pwm = 0.;
    int samples = 0;
    bool startup_finished = false;
    for (int i = 0; i < 1500; ++i) {
      const int pwm = rig.step(plant.counts());
      plant.step(pwm, .01);
      CHECK(pwm * desired >= 0 && !rig.control.stalled());
      if (!rig.control.startup()) startup_finished = true;
      else CHECK(!startup_finished);  // No volver a golpes de arranque durante la marcha.
      if (i >= 1000) { mean_speed += plant.velocity; mean_pwm += std::abs(pwm); ++samples; }
    }
    CHECK(startup_finished);
    mean_speed /= samples; mean_pwm /= samples;
    if (std::abs(mean_speed - desired) > std::fmax(4., std::abs(desired) * .20))
      std::cerr << "Planta, consigna=" << desired << ", media=" << mean_speed << ", PWM=" << mean_pwm << '\n';
    CHECK(std::abs(mean_speed - desired) <= std::fmax(4., std::abs(desired) * .20));
    CHECK(mean_pwm < 125.);  // No conservar el esfuerzo estático como velocidad de crucero.
  }
}
}  // namespace

int main(int argc, char** argv) {
  CHECK(argc == 2);
  const std::string scenario = argv[1];
  if (scenario == "reset_and_idle") reset_and_idle();
  else if (scenario == "startup_ramp") startup_ramp();
  else if (scenario == "two_counts_and_smooth_transfer") two_counts_and_smooth_transfer();
  else if (scenario == "wrong_encoder_direction_does_not_learn") wrong_encoder_direction_does_not_learn();
  else if (scenario == "zero_cancels_and_does_not_restart") zero_cancels_and_does_not_restart();
  else if (scenario == "stall_is_latched") stall_is_latched();
  else if (scenario == "no_repeated_startup_and_running_stall") no_repeated_startup_and_running_stall();
  else if (scenario == "reversal_and_target_flapping") reversal_and_target_flapping();
  else if (scenario == "zero_then_reverse_waits_for_rest") zero_then_reverse_waits_for_rest();
  else if (scenario == "pi_limits_and_antiwindup") pi_limits_and_antiwindup();
  else if (scenario == "quantized_friction_plant") quantized_friction_plant();
  else CHECK(false);
}
