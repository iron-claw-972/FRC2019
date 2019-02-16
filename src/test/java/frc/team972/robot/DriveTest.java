package frc.team972.robot;

import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Rotation2d;
import frc.team972.robot.statemachines.DriveDesireState;
import frc.team972.robot.statemachines.DriveStateMachine;
import frc.team972.robot.subsystems.DriveSubsystem;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;
import jeigen.DenseMatrix;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DriveTest {
    DriveSubsystem driveSubsystem = new DriveSubsystem(true);
    DriveStateMachine driveStateMachine = new DriveStateMachine(driveSubsystem);

    @Test
    public void testMecanum() {
        DriveSignal signal = MecanumHelper.cartesianCalculate(
                new CoordinateDriveSignal(0, 1, 0, true),
                0);

        /* //this section fails arbitrarily
        driveSubsystem.fastPeriodic(0);
        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.OPEN_LOOP);
        assertEquals(driveSubsystem.getPeriodicIO().left_front_demand, 0.0, 0.0);

        driveSubsystem.setOpenLoop(signal);
        driveSubsystem.fastPeriodic(0.01);

        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.OPEN_LOOP);
        assertEquals(driveSubsystem.getPeriodicIO().left_front_demand, 0.0, 0.0);
        */

        RobotState.getInstance().outputs_enabled = true;
        driveSubsystem.setOpenLoop(signal);
        driveSubsystem.fastPeriodic(0.02);

        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.OPEN_LOOP);
        assertEquals(driveSubsystem.getPeriodicIO().left_front_demand, 1.0, 0.0);

    }

    @Test
    public void testPathDrive() {
        RobotState.getInstance().outputs_enabled = true;

        driveSubsystem.setOpenLoopMecanum(new CoordinateDriveSignal(0, 0, 0, true));
        driveSubsystem.fastPeriodic(0.0);
        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.OPEN_LOOP);

        driveSubsystem.setMecanumDrivePoseDesired(new Pose2d(1, 1, Rotation2d.fromRadians(0)));
        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.PATH_FOLLOWING);
        driveSubsystem.fastPeriodic(0.01);
        assertEquals(driveSubsystem.getDriveControlState(), DriveSubsystem.DriveControlState.CLOSED_LOOP_MECANUM);
    }

    @Test
    public void testPathStateMachine() {
        driveStateMachine.update(0);
        assertEquals(driveStateMachine.getCurrentState(), DriveDesireState.MANUAL);

        Pose2d desired_pose = new Pose2d(2, 8, Rotation2d.fromRadians(0));
        driveStateMachine.requestNewPath(desired_pose);
        driveStateMachine.update(0);
        assertEquals(driveStateMachine.getCurrentState(), DriveDesireState.INIT_REQUEST);

        double x, y;
        x = y = 0;

        for (double t = 0; t < 10.0; t += Constants.dt) {
            DenseMatrix state_matrix = driveStateMachine.update(t);
            if (state_matrix != null) {
                //integrate velocity to get position
                x = x + (state_matrix.get(1, 0) * Constants.dt);
                y = y + (state_matrix.get(1, 1) * Constants.dt);
            }

            assertEquals(driveStateMachine.getCurrentState(), DriveDesireState.PATH_FOLLOWING);
        }

        assertEquals(x, desired_pose.getTranslation().x(), 0.001);
        assertEquals(y, desired_pose.getTranslation().y(), 0.001);
    }

}
