package frc.team972.robot;

import frc.team972.robot.subsystems.PistonClimb;

public class PistonClimbTest {

    public static void testPistonClimb(int time, PistonClimb.stageState currentStage)
    {

        if (currentStage == STAGE_2) {
            if (time <= (PistonClimb.getInstance().StageClimbTimings[1] - 1)) {
                PistonClimb.getInstance().range = PistonClimb.getInstance().HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES;
            } else {
                PistonClimb.getInstance().range = 1;
            }
        }

        if (currentStage == STAGE_5) {

            if (time <= (PistonClimb.getInstance().StageClimbTimings[4] - 1)) {
                PistonClimb.getInstance().range = PistonClimb.getInstance().GROUND_CLEARANCE + 2;
            } else {
                PistonClimb.getInstance().range = 1;
            }
        }

    }

}
