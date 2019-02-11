package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Rotation2d;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private DriveSubsystem drive_ = DriveSubsystem.getInstance();

    RobotStateEstimator() {
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    public void reset() {
        robot_state_.reset(0, new Pose2d());
    }

    @Override
    public void fastPeriodic(double timestamp) {
        double theta = Math.toRadians(drive_.getHeading());

        Pose2d last_pose = robot_state_.getLatestFieldToVehicle().getValue();

        DriveSubsystem.DriveSensorReading dsv = drive_.realUnitsToLinear(drive_.encoderToRealUnits(drive_.readEncodersVelocity()));
        dsv.left = dsv.left * Constants.dt;
        dsv.right = dsv.right * Constants.dt;
        dsv.left_back = dsv.left_back * Constants.dt;
        dsv.right_back = dsv.right_back * Constants.dt;

        double d_y_r = (dsv.right + dsv.left + dsv.right_back + dsv.left_back)/4.0;
        double d_x_r = (-dsv.right + dsv.left + dsv.right_back - dsv.left_back)/4.0;

        double d_x =(d_x_r * Math.cos(-theta) - d_y_r * Math.sin(-theta));
        double d_y = (d_x_r * Math.sin(-theta) + d_y_r * Math.cos(-theta));

        Pose2d v_pose = new Pose2d(d_x, d_y, new Rotation2d());
        Pose2d new_pose = new Pose2d(last_pose.transformBy(v_pose).getTranslation(), Rotation2d.fromRadians(theta));

        robot_state_.addFieldToVehicleObservation(timestamp, new_pose);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        System.out.println(robot_state_.getLatestFieldToVehicle().getValue());
    }

    @Override
    public void stop() {
        // No-op
    }
}