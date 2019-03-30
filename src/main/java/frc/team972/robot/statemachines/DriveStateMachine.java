package frc.team972.robot.statemachines;

import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.MotionProfileConstraints;
import frc.team972.robot.controls.MotionProfilePosition;
import frc.team972.robot.controls.TrapezodialMotionProfile;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Rotation2d;
import frc.team972.robot.lib.Translation2d;
import frc.team972.robot.subsystems.DriveSubsystem;
import frc.team972.robot.util.CoordinateDriveSignal;
import jeigen.DenseMatrix;

public class DriveStateMachine {
    private TrapezodialMotionProfile motionProfile;
    private DriveSubsystem drive_;

    private DrivePath path_;
    private RobotState robotState = RobotState.getInstance();

    MotionProfileConstraints constraints;

    public DriveStateMachine(DriveSubsystem drive) {
        drive_ = drive;
        constraints = new MotionProfileConstraints(0, 0);
        constraints.max_velocity = 50.0;
        constraints.max_acceleration = 50.0;
    }

    private DriveDesireState currentState = DriveDesireState.MANUAL;

    public DenseMatrix update(double t) {
        switch (currentState) {
            case MANUAL: {
                if (path_ != null) {
                    currentState = DriveDesireState.INIT_REQUEST;
                }
                break;
            }
            case INIT_REQUEST: {
                System.out.println("Initial drive_path detected");
                //Handle any logic associated with a new path request
                Translation2d start_translation = path_.desiredCoordinatePathStart.getTranslation();
                double path_dist_direct = start_translation.distance(path_.desiredCoordinatePathFinish.getTranslation());

                MotionProfilePosition goal = new MotionProfilePosition(path_dist_direct, 0);
                MotionProfilePosition start = new MotionProfilePosition(0, 0); //TODO: calculate actual real robot position at that current moment, rather than assume robot is stationary

                motionProfile = new TrapezodialMotionProfile(constraints, goal, start);

                currentState = DriveDesireState.PATH_FOLLOWING;
                break;
            }
            case PATH_FOLLOWING: {
                DenseMatrix desired_full_state = new DenseMatrix(2, 3);

                Translation2d start_translation = path_.desiredCoordinatePathStart.getTranslation();
                Translation2d end_translation = path_.desiredCoordinatePathFinish.getTranslation();

                double path_dist_direct = start_translation.distance(end_translation);
                double delta_t = (t * 0.001) - path_.startTime;

                MotionProfilePosition desired_mp = motionProfile.Calculate(delta_t);

                double d_x = end_translation.x() - start_translation.x();
                double d_x_c = (d_x / path_dist_direct) * desired_mp.position;
                double d_y = end_translation.y() - start_translation.y();
                double d_y_c = (d_y / path_dist_direct) * desired_mp.position;

                double v_x = (d_x / path_dist_direct) * desired_mp.velocity;
                double v_y = (d_y / path_dist_direct) * desired_mp.velocity;

                Pose2d desired_position = new Pose2d(d_x_c + start_translation.x(), d_y_c + start_translation.y(), Rotation2d.fromRadians(0));
                Pose2d desired_velocity = new Pose2d(v_x, v_y, Rotation2d.fromRadians(0));

                desired_full_state.set(0, 0, desired_position.getTranslation().x());
                desired_full_state.set(0, 1, desired_position.getTranslation().y());
                desired_full_state.set(0, 2, desired_position.getRotation().getRadians());
                desired_full_state.set(1, 0, desired_velocity.getTranslation().x());
                desired_full_state.set(1, 1, desired_velocity.getTranslation().y());
                desired_full_state.set(1, 2, desired_velocity.getRotation().getRadians());

                return desired_full_state;
            }
            default:
                currentState = DriveDesireState.MANUAL;
                break;
        }

        return null;
    }

    public void requestNewPath(Pose2d final_pose) {
        if (path_ == null) {
            double timestamp = System.currentTimeMillis() * 0.001;
            Pose2d current_pose = robotState.getLatestFieldToVehicle().getValue();
            path_ = new DrivePath(current_pose, final_pose, timestamp);
        }
    }

    public void requestManual() {
        currentState = DriveDesireState.MANUAL;
        path_ = null;
    }

    public DriveDesireState getCurrentState() {
        return currentState;
    }

    public DrivePath getPath_() {
        return path_;
    }

}

class DrivePath {
    public Pose2d desiredCoordinatePathStart;
    public Pose2d desiredCoordinatePathFinish;
    public double startTime;

    public DrivePath(Pose2d start, Pose2d end, double time) {
        desiredCoordinatePathStart = start;
        desiredCoordinatePathFinish = end;
        startTime = time;
    }
}
