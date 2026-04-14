// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>

/**
 * @brief Proportional-integral-derivative controller used by the trajectory controller.
 */
class PID {
 public:
  /**
   * @brief Creates a PID controller with the provided gains.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  PID(double Kp, double Ki, double Kd);

  /**
   * @brief Destroys the controller instance.
   */
  ~PID();

  /**
   * @brief Resets the integral state and derivative history.
   */
  void Reset();

  /**
   * @brief Clears only the accumulated integral term.
   */
  void ResetIntegral();

  /**
   * @brief Applies anti-windup back-calculation after output saturation.
   *
   * @param u_unsat Unsaturated controller output.
   * @param u_sat Saturated controller output.
   * @param dt Controller cycle time in seconds.
   * @param kaw Back-calculation gain.
   */
  void BackCalculate(double u_unsat, double u_sat, double dt, double kaw);

  /**
   * @brief Updates the controller gains.
   *
   * @param Kp Proportional gain.
   * @param Ki Integral gain.
   * @param Kd Derivative gain.
   */
  void SetParameters(double Kp, double Ki, double Kd);

  /**
   * @brief Calculates the PID output for the current error.
   *
   * @param e Control error.
   * @param dt Controller cycle time in seconds.
   * @return Controller output.
   */
  double Calc(double e, double dt);

 private:
  double Kp_ = 0.0;         ///< Proportional gain.
  double Ki_ = 0.0;         ///< Integral gain.
  double Kd_ = 0.0;         ///< Derivative gain.
  double i_val_ = 0.0;      ///< Accumulated integral term.
  double pre_e_ = nan("");  ///< Previously processed error, NaN before the first update.

};  // class PID
