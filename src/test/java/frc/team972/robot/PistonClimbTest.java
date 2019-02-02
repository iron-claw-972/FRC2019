package frc.team972.robot;

import frc.team972.robot.subsystems.PistonClimbSubsystem;

public class PistonClimbTest {

    public static void testPistonClimb(double time, PistonClimbSubsystem.stage currentStage)
    {

        if (currentStage == PistonClimbSubsystem.stage.STAGE_2) {
            if (time <= (PistonClimbSubsystem.getInstance().StageClimbTimings[1] - 1)) {
                PistonClimbSubsystem.getInstance().range = PistonClimbSubsystem.getInstance().HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES;
            } else {
                PistonClimbSubsystem.getInstance().range = 1;
            }
        }

        if (currentStage == PistonClimbSubsystem.stage.STAGE_5) {

            if (time <= (PistonClimbSubsystem.getInstance().StageClimbTimings[4] - 1)) {
                PistonClimbSubsystem.getInstance().range = PistonClimbSubsystem.getInstance().GROUND_CLEARANCE_INCHES + 2;
            } else {
                PistonClimbSubsystem.getInstance().range = 1;
            }
        }

    }

}
