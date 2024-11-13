/**
 * @file PID.cpp
 * @author Guido Küppers
 * @brief  Discrete PID-Controller Class.
 */
#include "PID.hpp"

PID::PID(double Kp, double Ki, double Kd) { this->SetParameters(Kp, Ki, Kd); }

void PID::SetParameters(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::Reset() {
  i_val_ = 0.0;
  pre_e_ = nan("");
}

double PID::Calc(double e, double dt) {
  i_val_ += e * dt;
  double d_val = 0.0;
  if (std::isnan(pre_e_)) {
    pre_e_ = e;
  }
  if (dt != 0.0) d_val = (e - pre_e_) / dt;

  pre_e_ = e;

  return Kp_ * e + Ki_ * i_val_ + Kd_ * d_val;
}
