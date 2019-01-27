package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.WristSubsystem;
import jeigen.DenseMatrix;

public class WristController {
    public StateSpacePlant plant_;
    public StateSpaceController controller_;
    public StateSpaceObserver observer_;

    public WristController(StateSpacePlant plant_, StateSpaceController controller_, StateSpaceObserver observer_) {
        this.plant_ = plant_;
        this.controller_ = controller_;
        this.observer_ = observer_;
    }

    public WristController() {
        plant_ = new StateSpacePlant(1, 3, 1);
        controller_ = new StateSpaceController(1, 3, 1);
        observer_ = new StateSpaceObserver(1, 3, 1);
    }

    public double getWrist_u() {
        return wrist_u;
    }

    private double wrist_u = 0.0;

    public MotionProfilePosition unprofiled_goal_ = new MotionProfilePosition(0 ,0);
    public MotionProfilePosition profiled_goal_ = new MotionProfilePosition(0 ,0);

    public void SetWeights(boolean second_stage) {
        //TODO: Switch out controller gains
        plant_.A_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_A);
        plant_.B_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_B);
        plant_.C_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_C);
        plant_.D_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_D);

        controller_.K_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_K_);
        controller_.Kff_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_Kff_);
        controller_.A_ = ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_A);

        observer_.L_ =  ControlsMathUtil.CloneMatrix(WristGains.first_stage_gain_L);

        //properly initialize plant via matrix copy and L matrix
        observer_.plant_.A_ = ControlsMathUtil.CloneMatrix(plant_.A_);
        observer_.plant_.B_ = ControlsMathUtil.CloneMatrix(plant_.B_);
        observer_.plant_.C_ = ControlsMathUtil.CloneMatrix(plant_.C_);
        observer_.plant_.D_ = ControlsMathUtil.CloneMatrix(plant_.D_);
        //preserve observer x matrix
    }

    public MotionProfilePosition UpdateProfiledGoal(boolean outputs_enabled) {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(Constants.kWristConstraints, unprofiled_goal_, profiled_goal_);

        if (outputs_enabled) {
            profiled_goal_ = profile.Calculate(5.0 * 0.001); // 5 ms
        } else {
            profiled_goal_ = profile.Calculate(0.0); //deded robot
        }

        return profiled_goal_;
    }

    public void SetGoal(double goal) {
        unprofiled_goal_.position = ControlsMathUtil.Cap(goal, 0, Constants.kWristMaxAngle);
        unprofiled_goal_.velocity = 0;
    }

    public double Update(WristSubsystem wristSubsystem) {
        HallCalibration hall_calibration = wristSubsystem.getHall_calibration_();
        boolean was_calibrated = hall_calibration.is_calibrated();

        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(0, 0, hall_calibration.Update(wristSubsystem.getEncoder(), wristSubsystem.getHall()));

        if (wristSubsystem.getHall_calibration_().is_calibrated) {
            SetWeights(observer_.plant_.x_.get(0, 0) >= 1.0);
        } else {
            SetWeights(false);
        }

        if (!wristSubsystem.isOutputs_enabled_()) {
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        if (hall_calibration.is_calibrated() && !was_calibrated) {
            observer_.plant_.x_.set(0,0, observer_.plant_.x_.get(0,0) + hall_calibration.offset);
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        UpdateProfiledGoal(wristSubsystem.isOutputs_enabled_());

        DenseMatrix r = new DenseMatrix(3, 1);
        r.set(0, 0, profiled_goal_.position);
        r.set(1, 0, profiled_goal_.velocity);
        r.set(2, 0, 0);

        controller_.r_ = r;

        wrist_u = controller_.Update(observer_.plant_.x_, controller_.r_).get(0, 0); //yay!

        if (!wristSubsystem.isOutputs_enabled_()) {
            wrist_u = ControlsMathUtil.Cap(wrist_u, -Constants.kWristVoltageCap, Constants.kWristVoltageCap);
        } else if (!hall_calibration.is_calibrated()) {
            wrist_u = Constants.kCalibrationVoltage;
        } else if (wristSubsystem.encoder_fault_detected_) {
            wrist_u = 2.0;
        } else if (profiled_goal_.position <= 1e-5) {
            wrist_u = 0.0;
        }

        if (wristSubsystem.old_pos_ == wristSubsystem.getEncoder() &&
                Math.abs(wrist_u) >= Constants.kEncoderFaultMinVoltage) {
            wristSubsystem.num_encoder_fault_ticks_++;
            if (wristSubsystem.num_encoder_fault_ticks_ > Constants.kEncoderFaultTicksAllowed) {
                wristSubsystem.encoder_fault_detected_ = true;
            }
        } else if (wristSubsystem.old_pos_ != wristSubsystem.getEncoder()) {
            wristSubsystem.num_encoder_fault_ticks_ = 0;
            wristSubsystem.encoder_fault_detected_ = false;
        }

        wrist_u = ControlsMathUtil.Cap(wrist_u, -Constants.kWristVoltageCap, Constants.kWristVoltageCap); //Cap... again?

        wristSubsystem.old_pos_ = wristSubsystem.getEncoder();

        DenseMatrix wrist_u_mat = new DenseMatrix(1, 1);
        wrist_u_mat.set(0, 0, wrist_u);
        observer_.Update(wrist_u_mat, y);
        //plant_.Update(wrist_u_mat); // we do not need to update our plant, we just use the input/output matrix

        if (wristSubsystem.isOutputs_enabled_()) {
            return wrist_u;
        } else {
            return 0.0;
        }
    }

}