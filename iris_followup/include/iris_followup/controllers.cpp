#include "PID.h"

class PDController {
private:
    PID pdController;

public:
    PDController() : pdController(PID::Builder().setKp(1.0).setKd(1.0).build()) {}

    double control(double setpoint, double measurement) {
        pdController.compute(setpoint, measurement);
        return pdController.getOutput();
    }
};

class CascadePDPI_FFController {
private:
    PID pdController;
    PID piController;

public:
    CascadePDPI_FFController() : pdController(PID::Builder().setKp(1.0).setKd(1.0).setFeedforwardEnabled(true).build()),
                                 piController(PID::Builder().setKp(1.0).setKi(1.0).build()) {}

    double control(double setpoint, double pd_measurement, double pi_measurement) {
        pdController.compute(setpoint, pd_measurement);
        double pd_output = pdController.getOutput();
        piController.compute(pd_output, pi_measurement);
        return piController.getOutput();
    }
};

class ParallelPDPI_FFController {
private:
    PID pdController;
    PID piController;
    PID ffController;

public:
    ParallelPDPI_FFController() : pdController(PID::Builder().setKp(1.0).setKd(1.0).build()),
                                  piController(PID::Builder().setKp(1.0).setKi(1.0).build()),
                                  ffController(PID::Builder().setKf(1.0).build()) {}

    double control(double setpoint, double measurement) {
        pdController.compute(setpoint, measurement);
        double pd_output = pdController.getOutput();
        piController.compute(setpoint, measurement);
        double pi_output = piController.getOutput();
        ffController.compute(setpoint, measurement);
        double ff_output = ffController.getOutput();
        return pd_output + pi_output + ff_output;
    }
};