#pragma once
#include <cmath>
#include <cstdint>

namespace robotracers {

// Entradas: cuentas físicas (positivo hacia delante), segundos y mm/s.
// No depende de Arduino, de red ni de periféricos. Una instancia por rueda.
struct WheelVelocityControl {
  static constexpr double MM_PER_COUNT = 3.14159265358979323846 * 23.0 / 59.94;
  static constexpr int PWM_MAX = 160;

  void reset(int64_t count, double now_s) {
    target_ = 0; reference_ = speed_ = eventSpeed_ = integral_ = 0;
    pwm_ = 0; active_ = starting_ = stalled_ = reversing_ = waitingForRest_ = false;
    previousCount_ = startupCount_ = count;
    previousTime_ = eventTime_ = progressTime_ = startupTime_ = now_s;
    startupPwm_ = 35; direction_ = 1;
  }

  void setTarget(int mm_s) {
    target_ = mm_s > 100 ? 100 : mm_s < -100 ? -100 : mm_s;
    if (target_ == 0) {
      reference_ = integral_ = 0; pwm_ = 0;
      active_ = starting_ = reversing_ = waitingForRest_ = false;
      // El fallo permanece enclavado; solo reset() permite un nuevo intento.
    }
  }

  void clearFault() {
    setTarget(0); stalled_ = false;
    // Nueva autorización explícita: conservar la velocidad de inercia medida.
  }

  int update(int64_t count, double now_s) {
    double dt = now_s - previousTime_;
    if (!std::isfinite(now_s) || dt <= 0) return pwm_;
    previousTime_ = now_s;
    if (dt > .05) dt = .05; // Un retraso de tarea nunca produce un salto de consigna.
    const int64_t delta = count - previousCount_;
    if (delta != 0) {
      const double eventDt = now_s - eventTime_;
      const double sample = double(delta) * MM_PER_COUNT / (eventDt > 1e-6 ? eventDt : 1e-6);
      const double alpha = eventDt / (.024 + eventDt);
      eventSpeed_ += alpha * (sample - eventSpeed_);
      eventTime_ = now_s; previousCount_ = count;
      if (active_ && delta * direction_ > 0) progressTime_ = now_s;
    }
    const double withoutEvent = now_s - eventTime_;
    const double cap = withoutEvent > 0 ? MM_PER_COUNT / withoutEvent : 1e9;
    speed_ = std::copysign(std::fmin(std::fabs(eventSpeed_), cap), eventSpeed_);
    if (target_ == 0 || stalled_) { pwm_ = 0; return 0; }

    const bool reversing = reference_ * target_ < 0;
    if (reversing && !reversing_) {
      // Reducir la consigna antes de cambiar PH; abandonar el boost anterior.
      reversing_ = true; starting_ = false; integral_ = 0;
    }
    if (!active_ && !waitingForRest_ && std::fabs(speed_) > 5.0 && withoutEvent < .25) {
      if (speed_ * target_ < 0) {
        // También después de un paquete cero: nunca aplicar impulso contrario
        // mientras las cuentas recientes muestran inercia en el otro sentido.
        reference_ = 0; waitingForRest_ = true;
      } else {
        // Si la rueda ya gira hacia la consigna, no repetir el arranque estático.
        direction_ = target_ < 0 ? -1 : 1;
        active_ = true; starting_ = false; integral_ = 0; progressTime_ = now_s;
      }
    }
    if (waitingForRest_) {
      pwm_ = 0;
      if (std::fabs(speed_) > 5.0 || withoutEvent < .10) return 0;
      waitingForRest_ = false; reversing_ = false;
    }
    const double wanted = reversing_ ? 0.0 : double(target_);
    reference_ = approach(reference_, wanted, 150.0 * dt);
    if (reference_ == 0) {
      pwm_ = 0; integral_ = 0; active_ = starting_ = false;
      if (reversing_) waitingForRest_ = true;
      return 0;
    }
    const int referenceDirection = reference_ < 0 ? -1 : 1;
    if (!active_) {
      direction_ = referenceDirection; active_ = starting_ = true;
      startupCount_ = count; startupTime_ = progressTime_ = now_s;
      startupPwm_ = 35; integral_ = 0;
      if (withoutEvent > .15) eventTime_ = now_s;
      pwm_ = direction_ * 35;
      return pwm_;
    }
    if ((starting_ && now_s - startupTime_ >= 5.0) || now_s - progressTime_ >= 5.0) {
      stalled_ = true; active_ = starting_ = false;
      reference_ = integral_ = 0; pwm_ = 0; return 0;
    }
    const double referenceMagnitude = std::fabs(reference_);
    const double error = referenceMagnitude - direction_ * speed_;
    const double feedforward = 35.0 + .125 * referenceMagnitude;
    if (starting_) {
      if ((count - startupCount_) * direction_ < 2) {
        startupPwm_ = std::fmin(double(PWM_MAX), 35.0 + 300.0 * (now_s - startupTime_));
        pwm_ = direction_ * int(std::lround(startupPwm_));
        return pwm_;
      }
      starting_ = false;
      // Transferencia validada: no conservar todo el esfuerzo de fricción
      // estática en el integrador. La rampa de PWM evita un escalón al salir.
      integral_ = limit(.40 * (std::fabs(double(pwm_)) - feedforward - .10 * error), -120., 125.);
    }
    const double free = feedforward + .10 * error + integral_;
    if ((free > 0 && free < PWM_MAX) || (free >= PWM_MAX && error < 0) || (free <= 0 && error > 0))
      integral_ = limit(integral_ + error * dt, -120.0, 125.0);
    double output = limit(feedforward + .10 * error + integral_, 0.0, double(PWM_MAX));
    if (reversing_) output = std::fmin(output, std::fabs(double(pwm_)));
    output = approach(std::fabs(double(pwm_)), output, 300.0 * dt);
    pwm_ = direction_ * int(std::lround(output));
    return pwm_;
  }

  int target() const { return target_; }
  double reference() const { return reference_; }
  double speed() const { return speed_; }
  int pwm() const { return pwm_; }
  bool startup() const { return starting_; }
  bool stalled() const { return stalled_; }

 private:
  static double limit(double value, double low, double high) {
    return value < low ? low : value > high ? high : value;
  }
  static double approach(double current, double target, double step) {
    return current < target ? std::fmin(current + step, target) : std::fmax(current - step, target);
  }
  int target_ = 0, pwm_ = 0, direction_ = 1;
  bool active_ = false, starting_ = false, stalled_ = false;
  bool reversing_ = false, waitingForRest_ = false;
  double reference_ = 0, speed_ = 0, eventSpeed_ = 0, integral_ = 0;
  double previousTime_ = 0, eventTime_ = 0, progressTime_ = 0, startupTime_ = 0, startupPwm_ = 35;
  int64_t previousCount_ = 0, startupCount_ = 0;
};

}  // namespace robotracers
