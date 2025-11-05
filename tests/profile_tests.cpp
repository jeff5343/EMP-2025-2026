#include "../include/util/trapezoid_profile.h"
#include "../include/util/profiled_pid.h"
#include "../include/util/angle.h"
#include <cstdio>

int main()
{
    TrapezoidProfile profile{TrapezoidProfile::Constraints{1, 1}};
    TrapezoidProfile::State current = TrapezoidProfile::State{0, 0};
    profile.setGoalState(TrapezoidProfile::State{3, 0});

    std::printf("\ninit: %.3f; goal: %.3f;\n",
                current.position, profile.getGoalState().position);
    double time = 0;
    while (current.position != profile.getGoalState().position)
    {
        time += 1.0;
        current = profile.calculate(1.0, current);
        std::printf("time: %.3f; setpoint: %.3f;\n", time, current.position);
    }

    current = TrapezoidProfile::State{3, 0};
    profile.setGoalState(TrapezoidProfile::State{0, 0});

    std::printf("\ninit: %.3f; goal: %.3f;\n",
                current.position, profile.getGoalState().position);
    time = 0;
    while (current.position != profile.getGoalState().position)
    {
        time += 1.0;
        current = profile.calculate(1.0, current);
        std::printf("time: %.3f; setpoint: %.3f;\n", time, current.position);
    }

    current = TrapezoidProfile::State{3, 1};
    profile.setGoalState(TrapezoidProfile::State{0, 0});

    std::printf("\ninit: %.3f; goal: %.3f;\n",
                current.position, profile.getGoalState().position);
    time = 0;
    while (current.position != profile.getGoalState().position)
    {
        time += 1.0;
        current = profile.calculate(1.0, current);
        std::printf("time: %.3f; setpoint: %.3f;\n", time, current.position);
    }

    std::printf("\nfunny\n");
    ProfiledPid controller{PidConstants{1, 0, 0}, {M_PI / 4.0, M_PI / 4.0}};

    controller.enableContinuousInput(true);
    // controller.reset(0);
    // controller.setGoal(3 * M_PI / 2.0);
    controller.reset(3 * M_PI / 2.0);
    controller.setGoal(0);
    while (controller.getSetpoint().position != controller.getGoalState().position)
    {
        std::printf("out: %.3f\n", Angle::toDegrees(controller.calculate(0)));
        for (int i = 0; i < 49; i++)
        {
            controller.calculate(0);
        }
    }
    std::printf("out: %.3f\n", Angle::toDegrees(controller.calculate(0)));
}