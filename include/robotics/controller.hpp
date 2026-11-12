#pragma once
// Joint controllers: speed motor (solver constraint) and P / PI / PD / PID / Relay (torque).

#include <iosfwd>

namespace robotics {

// Enum values are stored in .robot files: append new values at the end only.
// SpeedMotor preserves the original velocity-constraint actuator.
enum class ControllerType { SpeedMotor, P, PI, PD, PID, Relay };
enum class ControlVariable { Position, Velocity };
enum class ReferenceSignal { Constant, Step, Sine };

struct ControllerConfig {
    ControllerType type{ControllerType::SpeedMotor};
    ControlVariable variable{ControlVariable::Position};
    ReferenceSignal signal{ReferenceSignal::Constant};
    double target{};
    double amplitude{1};
    double frequency{0.25};
    double startTime{1};
    double kp{12};
    double ki{3};
    double kd{4};
    double integralLimit{10};      // Absolute I contribution, in Nm (not integral of error).
    double derivativeFilter{0.02}; // First order low-pass time constant in seconds.
    double deadband{0.02};         // Relay's zero-output band, in rad or rad/s.

    void validate() const;
    [[nodiscard]] double reference(double time) const;
};

struct ControllerState {
    double integral{};
    double filteredDerivative{};
    double previousMeasurement{};
    bool initialized{};
    // Measurement from a simulated sensor (written by robotics::updateSensors after every step). When
    // sensorFeedback is set the controller uses feedbackPosition / feedbackSpeed instead of the ideal joint
    // coordinate and speed; telemetry keeps the ideal values in ControlSample::angle / speed.
    bool sensorFeedback{};
    double feedbackPosition{};
    double feedbackSpeed{};
};

struct ControlSample {
    double time{};
    double target{};
    double measured{};
    double error{};
    double effort{};
    double p{};
    double i{};
    double d{};
    double angle{};
    double speed{};
    bool saturated{};
};

// Measurement derivative is physical joint speed for position control;
// velocity control estimates acceleration from consecutive samples instead.
// Advances `state`; callers may ignore the returned sample when only the state matters.
ControlSample evaluateController(const ControllerConfig& config, ControllerState& state, double time,
                                 double measurement, double measurementDerivative, double maxTorque,
                                 double dt);

void writeControlCsvHeader(std::ostream& out);
void writeControlCsvRow(std::ostream& out, int joint, const ControlSample& sample);

} // namespace robotics
