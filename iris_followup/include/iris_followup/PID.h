#ifndef PIDCLASS_LIBRARY_H
#define PIDCLASS_LIBRARY_H

#include <iostream>
#include <cmath>
#include <stdexcept>

const double PI = M_PI;
const double NEG_PI = -M_PI;

class PID {
private:
    double Kp, Ki, Kd, Kf;
    double dt;
    double estimated_process_variable, reference_process_variable;
    double error, last_error, integral_error, last_process_variable;
    double output, output_max, output_min;
    bool is_derivative_on_measurement;
    bool is_conditional_integration;
    bool is_feedforward_enabled;
    bool is_angular_input;

public:
    // Constructor
    PID();

    PID(double Kp, double Ki, double Kd, double Kf, double dt, double out_max, double out_min,
        bool derivative_on_measurement, bool conditional_integration, bool feedforward_enabled, bool angular_input);

    // Main PID compute function
    void compute(double new_reference_process_variable, double new_estimated_process_variable);

    // Reset function to clear errors and output
    void reset();

    // Enable or disable conditional_integration control
    void enableConditionalIntegration(bool use_conditional_integration);

    // Enable or disable feedforward control
    void enableFeedforward(bool use_feedforward);

    // Enable or disable angular input handling
    void enableAngularInput(bool angular_input);

    // Getters
    double getError() const;
    double getIntegralError() const;
    double getDerivativeError() const;
    double getOutput() const;
    double getInterval() const;
    void getParameters(double *Kp, double *Ki, double *Kd, double *Kf);

    // Setters
    void setParameters(double Kp, double Ki, double Kd, double Kf);
    void set_dt(double dt);
    void setOutputLimits(double out_max, double out_min);

    // Auxiliary methods
    double pythonModuloCompability(const double a, const double n);
    double normalize_angle(double angle, const double lower_bound = NEG_PI, const double upper_bound = PI,
                           const bool symmetric = false);
};

#endif //PIDCLASS_LIBRARY_H