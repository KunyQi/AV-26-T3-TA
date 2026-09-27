#pragma once
// Implement Controller so that, given only the target angle, the last
// measured angle, and the timestep, it drives the system to the target --
// despite whatever nonlinearity you identified from the CSVs.
//
// This is the file you submit. You can add private members, helper methods,
// filters, whatever your design needs. We will never run your internals.


#include "controller_interface.hpp"

#include <cmath>

class Controller : public IController {
public:
    double update(double target, double measured, double dt) override {
        if (!(dt > 0.0) || !std::isfinite(dt) ||
            !std::isfinite(target) || !std::isfinite(measured)) {
            return 0.0;
        }

        double error = target - measured;

        // Differentiate the measurement rather than the setpoint to avoid a
        // derivative kick when the requested angle changes. Low-pass filtering
        // reduces the effect of the sensor's 0.1-degree quantisation.
        if (havePreviousMeasurement_) {
            const double measuredRate = (measured - previousMeasurement_) / dt;
            const double alpha = dt / (kDerivativeFilterTime + dt);
            filteredMeasuredRate_ += alpha * (measuredRate - filteredMeasuredRate_);
        } else {
            havePreviousMeasurement_ = true;
        }
        previousMeasurement_ = measured;

        // The sensor only resolves 0.1 deg. Once the angle is within one
        // count and nearly stationary, stop integrating instead of making
        // tiny commands that can cause needless dithering around the target.
        if (std::fabs(error) <= kPositionTolerance &&
            std::fabs(filteredMeasuredRate_) <= kSettledRateTolerance) {
            integral_ = 0.0;
            return 0.0;
        }

        const double proposedIntegral = clampValue(
            integral_ + error * dt, -kIntegralLimit, kIntegralLimit);
        const double proposedOutput = kKp * error + kKi * proposedIntegral -
                                      kKd * filteredMeasuredRate_;

        // Conditional integration prevents windup against the output limit.
        // It still lets the integral unwind when the error points back into
        // the available command range.
        const bool pushingFurtherIntoSaturation =
            (proposedOutput > kMaxCommand && error > 0.0) ||
            (proposedOutput < -kMaxCommand && error < 0.0);
        if (!pushingFurtherIntoSaturation)
            integral_ = proposedIntegral;

        const double command = kKp * error + kKi * integral_ -
                               kKd * filteredMeasuredRate_;
        return clampValue(command, -kMaxCommand, kMaxCommand);
    }

    void reset() override {
        integral_ = 0.0;
        previousMeasurement_ = 0.0;
        filteredMeasuredRate_ = 0.0;
        havePreviousMeasurement_ = false;
    }

private:
    static double clampValue(double value, double low, double high) {
        return value < low ? low : (value > high ? high : value);
    }

    static constexpr double kKp = 2.5;                    // deg/s per deg
    static constexpr double kKi = 0.08;                   // 1/s^2
    static constexpr double kKd = 0.03;                   // dimensionless
    static constexpr double kMaxCommand = 15.0;           // deg/s
    static constexpr double kIntegralLimit = 10.0;        // deg*s
    static constexpr double kDerivativeFilterTime = 0.03; // seconds
    static constexpr double kPositionTolerance = 0.100001; // deg, one sensor count
    static constexpr double kSettledRateTolerance = 0.2;  // deg/s

    double integral_ = 0.0;
    double previousMeasurement_ = 0.0;
    double filteredMeasuredRate_ = 0.0;
    bool havePreviousMeasurement_ = false;
};
