package frc.team972.robot.subsystems;

import frc.team972.robot.RobotState;

public class RobotStateEstimator extends Subsystem {
    static RobotStateEstimator instance_ = new RobotStateEstimator();
    private RobotState robot_state_ = RobotState.getInstance();
    private DriveSubsystem drive_ = DriveSubsystem.getInstance();

    RobotStateEstimator() {
    }

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    @Override
    public void fastPeriodic() {
        DriveSubsystem.DriveSensorReading driveSensorVelocity = drive_.readEncodersVelocity();

    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        // No-op
    }

    @Override
    public void stop() {
        // No-op
    }
}