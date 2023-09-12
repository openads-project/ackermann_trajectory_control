/**
 * @file PID.hpp
 * @author Guido Küppers
 * @brief  Discrete PID-Controller Class.
 */
#pragma once

#include <cmath>

class PID
{
public:
    PID(double Kp, double Ki, double Kd);
    ~PID();

    void Reset();
    void SetParameters(double Kp, double Ki, double Kd);
    double Calc(double e, double dt);

private:
    double Kp_;
    double Ki_;
    double Kd_;
    double i_val_ = 0.0;
    double pre_e_ = nan("");

}; //class PID