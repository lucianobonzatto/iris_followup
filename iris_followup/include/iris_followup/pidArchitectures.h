#ifndef PIDARCHITECTURES_H
#define PIDARCHITECTURES_H

#include "PIDlib.h"

class PDController
{
private:
    PID pdController;

public:
    PDController() {}
    PDController(PID::Builder builder) : pdController(builder.build()) {}

    void update(double kp, double kd)
    {
        pdController.setKp(kp);
        pdController.setKd(kd);
    }

    void getParameters(double &kp, double &kd)
    {
        kp = pdController.getKp();
        kd = pdController.getKd();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
    }

    double control(double setpoint, double measurement)
    {
        pdController.compute(setpoint, measurement);
        return pdController.getOutput();
    }
};

class CascadePDPI_FFController
{
private:
    PID pdController;
    PID piController;

public:
    CascadePDPI_FFController(PID::Builder builder_pd, PID::Builder builder_pi)
        : pdController(builder_pd.build()),
          piController(builder_pi.build()) {}

    double control(double setpoint, double pd_measurement, double pi_measurement)
    {
        pdController.compute(setpoint, pd_measurement);
        double pd_output = pdController.getOutput();
        piController.compute(pd_output, pi_measurement);
        return piController.getOutput();
    }
};

class ParallelPDPIController
{
private:
    PID pdController;
    PID piController;

public:
    ParallelPDPIController() {}
    ParallelPDPIController(PID::Builder builder_pd, PID::Builder builder_pi)
        : pdController(builder_pd.build()),
          piController(builder_pi.build()) {}

    double control(double setpoint, double measurement)
    {
        pdController.compute(setpoint, measurement);
        double pd_output = pdController.getOutput();
        piController.compute(setpoint, measurement);
        double pi_output = piController.getOutput();
        return pd_output + pi_output;
    }

    void update(double kp_pd, double kd_pd, double kp_pi, double ki_pi)
    {
        pdController.setKp(kp_pd);
        pdController.setKd(kd_pd);
        piController.setKp(kp_pi);
        piController.setKi(ki_pi);
    }

    void getParameters(double &kp_pd, double &kd_pd, double &kp_pi, double &ki_pi)
    {
        kp_pd = pdController.getKp();
        kd_pd = pdController.getKd();
        kp_pi = pdController.getKp();
        ki_pi = pdController.getKi();
    }

    void setDT(double dt)
    {
        pdController.set_dt(dt);
        piController.set_dt(dt);
    }
};

#endif // PIDARCHITECTURES_H