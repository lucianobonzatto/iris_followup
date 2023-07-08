#include "tello_controllers.h"

int main() {
    // Create builders
    PID::Builder builder;

    // Create controllers
    TelloPDController pdController(builder, builder, builder, builder);
    TelloCascadePDPI_FFController cascadePDPI_FFController(builder, builder, builder, builder, builder, builder, builder, builder);
    TelloParallelPDPI_FFController parallelPDPI_FFController(builder, builder, builder, builder, builder, builder, builder, builder, builder, builder, builder, builder);

    // Create dummy pose and speed for testing
    Pose pose {0, 0, 0, 0};
    Speed speed {0, 0, 0, 0};

    // Use controllers
    Speed pdResult = pdController.control(pose, pose);
    Speed cascadeResult = cascadePDPI_FFController.control(pose, pose, speed);
    Speed parallelResult = parallelPDPI_FFController.control(pose, pose);

    return 0;
}
