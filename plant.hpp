#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle using the lag and backlash identified from the logs.

#include <cmath>

struct Plant {
    // Output angle, motor-side angle, and filtered motor velocity.
    double angle = 0.0;
    double motorAngle = 0.0;
    double motorVelocity = 0.0;

    static constexpr double kGain = 1.3;
    static constexpr double kTimeConstant = 0.07;
    static constexpr double kBacklashHalfWidth = 2.5;

    // u_cmd : commanded velocity, deg/s
    // dt    : timestep, seconds
    // return: measured output angle, deg
    double step(double u_cmd, double dt) {
        if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(u_cmd))
            return std::round(angle / 0.1) * 0.1;

        // The commanded rate drives a first-order motor-side velocity.
        // Integrate that response exactly over this tick so the model remains
        // well behaved when the control timestep changes.
        const double oldVelocity = motorVelocity;
        const double targetVelocity = kGain * u_cmd;
        const double decay = std::exp(-dt / kTimeConstant);
        motorVelocity = targetVelocity + (oldVelocity - targetVelocity) * decay;
        motorAngle += targetVelocity * dt +
                      (oldVelocity - targetVelocity) * kTimeConstant * (1.0 - decay);

        // Backlash: the motor can move within +/- 2.5 degrees of the output
        // without moving it. A direction reversal therefore takes up 5 deg
        // of motor travel before the output follows again.
        const double relativeAngle = motorAngle - angle;
        if (relativeAngle > kBacklashHalfWidth)
            angle = motorAngle - kBacklashHalfWidth;
        else if (relativeAngle < -kBacklashHalfWidth)
            angle = motorAngle + kBacklashHalfWidth;

        return std::round(angle / 0.1) * 0.1;  // the sensor reads to 0.1 deg
    }

    void reset() {
        angle = 0.0;
        motorAngle = 0.0;
        motorVelocity = 0.0;
    }
};
