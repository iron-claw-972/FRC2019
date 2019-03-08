package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.RobotStateEstimator;
import jeigen.DenseMatrix;

public class DriveMotorController {

    public StateSpacePlant plant_;
    public StateSpaceController controller_;
    public StateSpaceObserver observer_;

    MotionProfilePosition unprofiled_goal_ = new MotionProfilePosition(0, 0);

    public DriveMotorController() {
        plant_ = new StateSpacePlant(1, 3, 1);
        controller_ = new StateSpaceController(1, 3, 1);
        observer_ = new StateSpaceObserver(1, 3, 1);
    }

    public double getMotor_u() {
        return motor_u;
    }

    private double motor_u = 0.0;

    public void SetWeights() {
        plant_.A_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.A());
        plant_.B_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.B());
        plant_.C_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.C());
        plant_.D_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.D());

        controller_.K_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.K());
        controller_.Kff_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.Kff());
        controller_.A_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.A());

        observer_.L_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.L());

        observer_.plant_.A_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.A());
        observer_.plant_.B_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.B());
        observer_.plant_.C_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.C());
        observer_.plant_.D_ = ControlsMathUtil.CloneMatrix(DriveMotorGains.D());
    }

    public void SetGoal(MotionProfilePosition unprofiled_goal_) {
        this.unprofiled_goal_ = unprofiled_goal_;
    }

    @SuppressWarnings("Duplicates")
    public double Update(double motor_encoder_radian_value) {
        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(0, 0, motor_encoder_radian_value);

        SetWeights();

        DenseMatrix r = new DenseMatrix(3, 1);
        r.set(0, 0, unprofiled_goal_.position);
        r.set(1, 0, unprofiled_goal_.velocity);
        r.set(2, 0, 0);
        controller_.r_ = r;

        motor_u = controller_.Update(observer_.plant_.x_, controller_.r_).get(0, 0);
        motor_u = ControlsMathUtil.Cap(motor_u, -Constants.kDriveVoltageCap, Constants.kDriveVoltageCap);

        DenseMatrix motor_u_mat = new DenseMatrix(1, 1);
        motor_u_mat.set(0, 0, motor_u);
        observer_.Update(motor_u_mat, y);

        if (!RobotState.getInstance().outputs_enabled) {
            motor_u = 0.0;
        }

        return motor_u;
    }


}
