#include "robotics/controller.hpp"

#include "robotics/math.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>

namespace robotics {

void ControllerConfig::validate() const {
    if (type < ControllerType::SpeedMotor || type > ControllerType::Relay
        || variable < ControlVariable::Position || variable > ControlVariable::Velocity
        || signal < ReferenceSignal::Constant || signal > ReferenceSignal::Sine) {
        throw std::runtime_error("Unknown controller, variable or reference signal");
    }
    for (double value :
         {target, amplitude, frequency, startTime, kp, ki, kd, integralLimit, derivativeFilter, deadband}) {
        if (!std::isfinite(value)) {
            throw std::runtime_error("Controller parameters must be finite");
        }
    }
    if (frequency < 0 || startTime < 0 || kp < 0 || ki < 0 || kd < 0 || integralLimit < 0
        || derivativeFilter < 0 || deadband < 0) {
        throw std::runtime_error("Gains, limits, filter, frequency, delay and deadband must be nonnegative");
    }
}

double ControllerConfig::reference(double time) const {
    if (signal == ReferenceSignal::Constant || time < startTime) {
        return target;
    }
    if (signal == ReferenceSignal::Step) {
        return target + amplitude;
    }
    return target + amplitude * std::sin(2 * pi * frequency * (time - startTime));
}

ControlSample evaluateController(const ControllerConfig& config, ControllerState& state, double time,
                                 double measurement, double measurementDerivative, double maxTorque,
                                 double dt) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(maxTorque) || maxTorque < 0 || !std::isfinite(time)
        || !std::isfinite(measurement) || !std::isfinite(measurementDerivative)) {
        throw std::runtime_error("Invalid controller timestep or measurement");
    }
    ControlSample sample;
    sample.time = time;
    sample.target = config.reference(time);
    sample.measured = measurement;
    sample.error = sample.target - measurement;
    if (!std::isfinite(sample.target) || !std::isfinite(sample.error)) {
        throw std::runtime_error("Controller reference overflow");
    }

    // Filtered derivative of the measurement (not of the error).
    double derivative = config.variable == ControlVariable::Position
                            ? measurementDerivative
                            : (state.initialized ? (measurement - state.previousMeasurement) / dt : 0);
    double alpha = dt / (config.derivativeFilter + dt);
    state.filteredDerivative += alpha * (derivative - state.filteredDerivative);
    state.previousMeasurement = measurement;
    state.initialized = true;

    if (config.type == ControllerType::Relay) {
        state.integral = 0;
        sample.effort =
            sample.error > config.deadband ? maxTorque : (sample.error < -config.deadband ? -maxTorque : 0);
        sample.saturated = maxTorque > 0 && sample.effort != 0;
        return sample;
    }
    if (config.type == ControllerType::SpeedMotor) {
        state.integral = 0;
        return sample;
    }

    bool hasI = config.type == ControllerType::PI || config.type == ControllerType::PID;
    bool hasD = config.type == ControllerType::PD || config.type == ControllerType::PID;
    sample.p = config.kp * sample.error;
    // Derivative on measurement avoids a torque spike at a reference step.
    sample.d = hasD ? -config.kd * state.filteredDerivative : 0;
    if (hasI && maxTorque > 0) {
        double candidate = std::clamp(state.integral + config.ki * sample.error * dt, -config.integralLimit,
                                      config.integralLimit);
        double raw = sample.p + candidate + sample.d;
        double change = candidate - state.integral;
        // Conditional integration: allow unwinding, reject growth into saturation.
        if (!((raw > maxTorque && change > 0) || (raw < -maxTorque && change < 0))) {
            state.integral = candidate;
        }
    } else {
        state.integral = 0;
    }
    sample.i = state.integral;
    double raw = sample.p + sample.i + sample.d;
    if (!std::isfinite(raw)) {
        throw std::runtime_error("Controller output overflow; reduce gains or target");
    }
    sample.effort = std::clamp(raw, -maxTorque, maxTorque);
    sample.saturated = std::abs(raw) > maxTorque;
    return sample;
}

void writeControlCsvHeader(std::ostream& out) {
    out << "time,joint_id,target,measured,error,torque_nm,p_nm,i_nm,d_nm,angle_rad,speed_rad_s,saturated\n";
}

void writeControlCsvRow(std::ostream& out, int joint, const ControlSample& sample) {
    out << std::setprecision(17) << sample.time << ',' << joint << ',' << sample.target << ','
        << sample.measured << ',' << sample.error << ',' << sample.effort << ',' << sample.p << ','
        << sample.i << ',' << sample.d << ',' << sample.angle << ',' << sample.speed << ','
        << sample.saturated << '\n';
}

} // namespace robotics
