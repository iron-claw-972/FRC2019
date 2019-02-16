package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.IntakeSubsystem;
import jeigen.DenseMatrix;

public class IntakeController {
    public StateSpacePlant plant_;
    public StateSpaceController controller_;
    public StateSpaceObserver observer_;

    public IntakeController(StateSpacePlant plant_, StateSpaceController controller_, StateSpaceObserver observer_) {
        this.plant_ = plant_;
        this.controller_ = controller_;
        this.observer_ = observer_;
    }

    public IntakeController() {
        plant_ = new StateSpacePlant(1, 3, 1);
        controller_ = new StateSpaceController(1, 3, 1);
        observer_ = new StateSpaceObserver(1, 3, 1);
    }

    public double getIntake_u() {
        return intake_u;
    }

    private double intake_u = 0.0;

    public MotionProfilePosition unprofiled_goal_ = new MotionProfilePosition(0 ,0);
    public MotionProfilePosition profiled_goal_ = new MotionProfilePosition(0 ,0);

    public void SetWeights() {
        plant_.A_ = ControlsMathUtil.CloneMatrix(IntakeGains.A());
        plant_.B_ = ControlsMathUtil.CloneMatrix(IntakeGains.B());
        plant_.C_ = ControlsMathUtil.CloneMatrix(IntakeGains.C());
        plant_.D_ = ControlsMathUtil.CloneMatrix(IntakeGains.D());

        controller_.K_ = ControlsMathUtil.CloneMatrix(IntakeGains.K());
        controller_.Kff_ = ControlsMathUtil.CloneMatrix(IntakeGains.Kff());
        controller_.A_ = ControlsMathUtil.CloneMatrix(IntakeGains.A());

        observer_.L_ =  ControlsMathUtil.CloneMatrix(IntakeGains.L());

        observer_.plant_.A_ = ControlsMathUtil.CloneMatrix(IntakeGains.A());
        observer_.plant_.B_ = ControlsMathUtil.CloneMatrix(IntakeGains.B());
        observer_.plant_.C_ = ControlsMathUtil.CloneMatrix(IntakeGains.C());
        observer_.plant_.D_ = ControlsMathUtil.CloneMatrix(IntakeGains.D());
        //preserve observer x matrix
    }

    public MotionProfilePosition UpdateProfiledGoal(boolean outputs_enabled) {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(Constants.kIntakeConstraints, unprofiled_goal_, profiled_goal_);

        if (outputs_enabled) {
            profiled_goal_ = profile.Calculate(Constants.dt);
        } else {
            profiled_goal_ = profile.Calculate(0.0);
        }

        return profiled_goal_;
    }

    public void SetGoal(double goal) {
        unprofiled_goal_.position = ControlsMathUtil.Cap(goal, 0, Constants.kIntakeMaxAngle);
        unprofiled_goal_.velocity = 0;
    }

    @SuppressWarnings("Duplicates")
    public double Update(IntakeSubsystem IntakeSubsystem) {
        HallCalibration hall_calibration = IntakeSubsystem.getHall_calibration_();
        boolean was_calibrated = hall_calibration.is_calibrated();

        //TODO: add velocity component to the observer matrix [pos, vel] so simple 1x2 matrix needed
        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(0, 0, hall_calibration.Update(IntakeSubsystem.getEncoder(), IntakeSubsystem.getHall()));

        //Gain Schedule
        SetWeights();

        if (!IntakeSubsystem.isOutputs_enabled_()) {
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        if (hall_calibration.is_calibrated() && !was_calibrated) {
            observer_.plant_.x_.set(0,0, observer_.plant_.x_.get(0,0) + hall_calibration.offset);
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        UpdateProfiledGoal(IntakeSubsystem.isOutputs_enabled_());

        DenseMatrix r = new DenseMatrix(3, 1);
        r.set(0, 0, profiled_goal_.position);
        r.set(1, 0, profiled_goal_.velocity);
        r.set(2, 0, 0);

        controller_.r_ = r;

        intake_u = controller_.Update(observer_.plant_.x_, controller_.r_).get(0, 0);

        if (!hall_calibration.is_calibrated()) {
            //todo: perhaps manual control to allow for manual zeroing?
            intake_u = Constants.kCalibrationVoltage;
        } else if (Math.abs(profiled_goal_.position) <= 0.01) {
            //intake_u = 0.0;
            //Ignore for now.
        }

        intake_u = ControlsMathUtil.Cap(intake_u, -Constants.kIntakeVoltageCap, Constants.kIntakeVoltageCap);

        DenseMatrix intake_u_mat = new DenseMatrix(1, 1);
        intake_u_mat.set(0, 0, intake_u);
        observer_.Update(intake_u_mat, y);

        if (!IntakeSubsystem.isOutputs_enabled_()) {
            intake_u = 0.0;
        }

        return intake_u;
    }

}