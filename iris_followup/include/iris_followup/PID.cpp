#include "PID.h"

PID::PID()
    : Kp(0), Ki(0), Kd(0), Kf(0), dt(0), output_max(0), output_min(0),
      is_derivative_on_measurement(0), is_conditional_integration(0),
      is_feedforward_enabled(0), is_angular_input(0), reference_process_variable(0),
      estimated_process_variable(0), integral_error(0), error(0), last_error(0), last_process_variable(0), output(0)
{
}

PID::PID(double Kp, double Ki, double Kd, double Kf, double dt, double out_max, double out_min,
         bool derivative_on_measurement, bool conditional_integration, bool feedforward_enabled, bool angular_input)
    : Kp(Kp), Ki(Ki), Kd(Kd), Kf(Kf), dt(dt), output_max(out_max), output_min(out_min),
      is_derivative_on_measurement(derivative_on_measurement), is_conditional_integration(conditional_integration),
      is_feedforward_enabled(feedforward_enabled), is_angular_input(angular_input), reference_process_variable(0),
      estimated_process_variable(0), integral_error(0), error(0), last_error(0), last_process_variable(0), output(0)
{
    if (Kp < 0 || Ki < 0 || Kd < 0 || Kf < 0)
    {
        throw std::invalid_argument("The controller gains should be positive.");
    }

    if (dt <= 0)
    {
        throw std::invalid_argument("dt must be positive and non-zero.");
    }
    if (out_min >= out_max)
    {
        throw std::invalid_argument("out_min must be less than out_max.");
    }
}

void PID::compute(double new_reference_process_variable, double new_estimated_process_variable)
{
    double differential_error, temporary_sum_error;
    double computed_output;

    if (is_angular_input)
    {
        new_reference_process_variable = normalize_angle(new_reference_process_variable);
        new_estimated_process_variable = normalize_angle(new_estimated_process_variable);
    }

    reference_process_variable = new_reference_process_variable;
    estimated_process_variable = new_estimated_process_variable;

    error = reference_process_variable - estimated_process_variable;
    // Calculate the derivative of the error or measurement
    if (is_derivative_on_measurement)
    {
        differential_error = (estimated_process_variable - last_process_variable) / dt;
    }
    else
    {
        differential_error = (error - last_error) / dt;
    }

    // Integral of the error
    temporary_sum_error = integral_error + error * dt;

    // Calculate PID output with or without feedforward term
    if (is_feedforward_enabled)
    {
        computed_output = Kp * error + Ki * temporary_sum_error + Kd * differential_error + Kf * reference_process_variable;
    }
    else
    {
        computed_output = Kp * error + Ki * temporary_sum_error + Kd * differential_error;
    }

    // Anti-windup: back calculation or conditional integration
    if (is_conditional_integration)
    {
        // Conditional integration
        if ((computed_output < output_max && computed_output > output_min) || (computed_output >= output_max && error < 0) || (computed_output <= output_min && error > 0))
        {
            integral_error = temporary_sum_error;
        }
    }
    else
    {
        // Back-calculation
        if (Ki != 0)
        {
            integral_error = (computed_output - Kp * error - Kd * differential_error - (is_feedforward_enabled ? Kf * reference_process_variable : 0)) / Ki;
        }
        else
        {
            integral_error = 0;
        }
    }

    // Saturate output after anti-windup
    if (computed_output > output_max)
    {
        computed_output = output_max;
    }
    else if (computed_output < output_min)
    {
        computed_output = output_min;
    }

    // Storing error and process variable last measurement
    last_error = error;
    last_process_variable = estimated_process_variable;

    // Storing last controller output
    output = computed_output;
}

void PID::reset()
{
    reference_process_variable = 0;
    estimated_process_variable = 0;
    integral_error = 0;
    error = 0;
    last_error = 0;
    last_process_variable = 0;
    output = 0;
}

void PID::enableConditionalIntegration(bool use_conditional_integration)
{
    is_conditional_integration = use_conditional_integration;
}

void PID::enableAngularInput(bool angular_input)
{
    is_angular_input = angular_input;
}

void PID::enableFeedforward(bool use_feedforward)
{
    is_feedforward_enabled = use_feedforward;
}

double PID::getError() const
{
    return error;
}

double PID::getIntegralError() const
{
    return integral_error;
}

double PID::getDerivativeError() const
{
    return (error - last_error) / dt;
}

double PID::getOutput() const
{
    return output;
}

void PID::setParameters(double Kp, double Ki, double Kd, double Kf)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Kf = Kf;
}

void PID::getParameters(double *Kp, double *Ki, double *Kd, double *Kf)
{
    *Kp = this->Kp;
    *Ki = this->Ki;
    *Kd = this->Kd;
    *Kf = this->Kf;
}

void PID::set_dt(double dt)
{
    if (dt <= 0)
    {
        throw std::invalid_argument("dt must be positive and non-zero.");
    }
    this->dt = dt;
}

void PID::setOutputLimits(double out_max, double out_min)
{
    if (out_min >= out_max)
    {
        throw std::invalid_argument("out_min must be less than out_max.");
    }
    this->output_max = out_max;
    this->output_min = out_min;
}

double PID::pythonModuloCompability(const double a, const double n)
{
    return (a - (n * floor(a / n)));
}

double PID::normalize_angle(double angle, const double lower_bound, const double upper_bound, const bool symmetric)
{

    if (lower_bound >= upper_bound)
    {
        throw std::invalid_argument("lower_bound must be less than upper_bound.");
    }

    double range = upper_bound - lower_bound;
    double offset = symmetric ? (upper_bound + lower_bound) / 2.0 : lower_bound;

    // Normalize to the range [0, range)
    angle = pythonModuloCompability(angle - offset, range);

    // Shift to the desired interval
    angle += offset;

    return angle;
}

double PID::getInterval() const
{
    return dt;
}
