package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import jeigen.DenseMatrix;

public class ElevatorController {
    public StateSpacePlant plant_;
    public StateSpaceController controller_;
    public StateSpaceObserver observer_;

    public ElevatorController(StateSpacePlant plant_, StateSpaceController controller_, StateSpaceObserver observer_) {
        this.plant_ = plant_;
        this.controller_ = controller_;
        this.observer_ = observer_;
    }

    public ElevatorController() {
        plant_ = new StateSpacePlant(1, 3, 1);
        controller_ = new StateSpaceController(1, 3, 1);
        observer_ = new StateSpaceObserver(1, 3, 1);
    }

    public double getElevator_u() {
        return elevator_u;
    }

    private double elevator_u = 0.0;

    public MotionProfilePosition unprofiled_goal_ = new MotionProfilePosition(0, 0);
    public MotionProfilePosition profiled_goal_ = new MotionProfilePosition(0, 0);

    public void SetWeights(boolean second_stage) {
        plant_.A_ = ControlsMathUtil.CloneMatrix(ElevatorGains.A());
        plant_.B_ = ControlsMathUtil.CloneMatrix(ElevatorGains.B());
        plant_.C_ = ControlsMathUtil.CloneMatrix(ElevatorGains.C());
        plant_.D_ = ControlsMathUtil.CloneMatrix(ElevatorGains.D());

        controller_.K_ = ControlsMathUtil.CloneMatrix(ElevatorGains.K());
        controller_.Kff_ = ControlsMathUtil.CloneMatrix(ElevatorGains.Kff());
        controller_.A_ = ControlsMathUtil.CloneMatrix(ElevatorGains.A());

        observer_.L_ = ControlsMathUtil.CloneMatrix(ElevatorGains.L());

        observer_.plant_.A_ = ControlsMathUtil.CloneMatrix(ElevatorGains.A());
        observer_.plant_.B_ = ControlsMathUtil.CloneMatrix(ElevatorGains.B());
        observer_.plant_.C_ = ControlsMathUtil.CloneMatrix(ElevatorGains.C());
        observer_.plant_.D_ = ControlsMathUtil.CloneMatrix(ElevatorGains.D());
    }

    public MotionProfilePosition UpdateProfiledGoal(boolean outputs_enabled) {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(Constants.kElevatorConstraints, unprofiled_goal_, profiled_goal_);

        if (outputs_enabled) {
            profiled_goal_ = profile.Calculate(Constants.dt);
        } else {
            profiled_goal_ = profile.Calculate(0.0); //deded robot
        }

        return profiled_goal_;
    }

    public void SetGoal(double goal) {
        unprofiled_goal_.position = ControlsMathUtil.Cap(goal, 0, Constants.kElevatorMaxHeight);
        unprofiled_goal_.velocity = 0;
    }

    @SuppressWarnings("Duplicates")
    public double Update(ElevatorSubsystem elevatorSubsystem) {
        HallCalibration hall_calibration = elevatorSubsystem.getHall_calibration_();
        boolean was_calibrated = hall_calibration.is_calibrated();

        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(0, 0, hall_calibration.Update(elevatorSubsystem.getEncoder(), elevatorSubsystem.getHall()));

        //Gain Schedule
        SetWeights(false);

        if (!elevatorSubsystem.isOutputs_enabled_()) {
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        if (hall_calibration.is_calibrated() && !was_calibrated) {
            observer_.plant_.x_.set(0, 0, observer_.plant_.x_.get(0, 0) + hall_calibration.offset);
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        UpdateProfiledGoal(elevatorSubsystem.isOutputs_enabled_());

        DenseMatrix r = new DenseMatrix(3, 1);
        r.set(0, 0, profiled_goal_.position);
        r.set(1, 0, profiled_goal_.velocity);
        r.set(2, 0, 0);

        controller_.r_ = r;

        elevator_u = controller_.Update(observer_.plant_.x_, controller_.r_).get(0, 0); //yay!

        /*
        if (!hall_calibration.is_calibrated()) {
            elevator_u = Constants.kCalibrationVoltage;
        } else if (elevatorSubsystem.encoder_fault_detected_) {
            elevator_u = 2.0;
        }
        */

        if(r.get(0,0) < 0.005) {
            elevator_u = 0.0;
        }

        elevator_u = ControlsMathUtil.Cap(elevator_u, -Constants.kElevatorVoltageCap, Constants.kElevatorVoltageCap);

        /* // ---- ENCODER FAULT DETECTION REMOVED TEMPORARILY
        if (elevatorSubsystem.old_pos_ == elevatorSubsystem.getEncoder() &&
                Math.abs(elevator_u) >= Constants.kEncoderFaultMinVoltage) {
            elevatorSubsystem.num_encoder_fault_ticks_++;
            if (elevatorSubsystem.num_encoder_fault_ticks_ > Constants.kEncoderFaultTicksAllowed) {
                elevatorSubsystem.encoder_fault_detected_ = true;
            }
        } else if (elevatorSubsystem.old_pos_ != elevatorSubsystem.getEncoder()) {
            elevatorSubsystem.num_encoder_fault_ticks_ = 0;
            elevatorSubsystem.encoder_fault_detected_ = false;
        }

        elevatorSubsystem.old_pos_ = elevatorSubsystem.getEncoder();
        */

        DenseMatrix elevator_u_mat = new DenseMatrix(1, 1);
        elevator_u_mat.set(0, 0, elevator_u);
        observer_.Update(elevator_u_mat, y);

        if (!elevatorSubsystem.isOutputs_enabled_()) {
            elevator_u = 0.0;
        }

        return elevator_u;
    }

}