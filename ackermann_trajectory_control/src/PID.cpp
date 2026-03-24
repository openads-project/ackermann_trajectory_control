// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

/**
 * @file PID.cpp
 * @brief Implements the PID controller used by the Ackermann trajectory controller.
 */
#include "PID.hpp"

PID::PID(double Kp, double Ki, double Kd) { this->SetParameters(Kp, Ki, Kd); }

PID::~PID() = default;

void PID::SetParameters(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PID::Reset() {
  i_val_ = 0.0;
  pre_e_ = nan("");
}

void PID::ResetIntegral() { i_val_ = 0.0; }

void PID::BackCalculate(double u_unsat, double u_sat, double dt, double kaw) {
  if (dt <= 0.0 || kaw <= 0.0 || Ki_ == 0.0) return;
  i_val_ += (kaw / Ki_) * (u_sat - u_unsat) * dt;
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
