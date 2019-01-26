package frc.team972.robot.subsystems.controller;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.subsystems.ElevatorSubsystem;
import jeigen.DenseMatrix;

public class ElevatorController {
    public StateSpacePlant plant_ = new StateSpacePlant(1, 3, 1);
    public StateSpaceController controller_ = new StateSpaceController(1, 3, 1);

    public StateSpaceObserver observer_ = new StateSpaceObserver(1, 3, 1);

    public double getElevator_u() {
        return elevator_u;
    }

    private double elevator_u = 0.0;

    public MotionProfilePosition unprofiled_goal_ = new MotionProfilePosition(0 ,0);
    public MotionProfilePosition profiled_goal_ = new MotionProfilePosition(0 ,0);

    public void SetWeights(boolean second_stage) {
        //TODO: Switch out controller gains
        plant_.A_ = new DenseMatrix("1.0 0.004791236347425109 5.181376400930422e-05; 0.0 0.917673229771176 0.020432962307718957; 0.0 0.0 1.0");
        plant_.B_ = new DenseMatrix("5.181376400930422e-05; 0.020432962307718957; 0.0");
        plant_.C_ = new DenseMatrix("1.0 0.0 0.0");
        plant_.D_ = new DenseMatrix("0.0");

        controller_.K_ = new DenseMatrix("16.41399515492959 -0.04285978261048596 1.0");
        controller_.Kff_ = new DenseMatrix("0.0 48.94052976460639 0.0");
        controller_.A_ = ControlsMathUtil.CloneMatrix(plant_.A_);

        //properly initialize plant via matrix copy and L matrix
        observer_.plant_.A_ = ControlsMathUtil.CloneMatrix(plant_.A_);
        observer_.plant_.B_ = ControlsMathUtil.CloneMatrix(plant_.B_);
        observer_.plant_.C_ = ControlsMathUtil.CloneMatrix(plant_.C_);
        observer_.plant_.D_ = ControlsMathUtil.CloneMatrix(plant_.D_);
        //preserve observer x matrix

        observer_.L_ = new DenseMatrix("0.46742789528315815; 14.840398277521956; 104.89633760328563");
    }

    public MotionProfilePosition UpdateProfiledGoal(boolean outputs_enabled) {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(Constants.kElevatorConstraints, unprofiled_goal_, profiled_goal_);

        if (outputs_enabled) {
            profiled_goal_ = profile.Calculate(5.0 * 0.001); // 5 ms
        } else {
            profiled_goal_ = profile.Calculate(0.0); //deded robot
        }

        return profiled_goal_;
    }

    public void SetGoal(double goal) {
        unprofiled_goal_.position = ControlsMathUtil.Cap(goal, 0, Constants.kElevatorMaxHeight);
        unprofiled_goal_.velocity = 0;
    }

    public double Update(ElevatorSubsystem elevatorSubsystem) {
        HallCalibration hall_calibration = elevatorSubsystem.getHall_calibration_();
        boolean was_calibrated = hall_calibration.is_calibrated();

        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(0, 0, hall_calibration.Update(elevatorSubsystem.getEncoder(), elevatorSubsystem.getHall()));

        if (elevatorSubsystem.getHall_calibration_().is_calibrated) {
            SetWeights(observer_.plant_.x_.get(0, 0) >= 1.0);
        } else {
            SetWeights(false);
        }

        if (!elevatorSubsystem.isOutputs_enabled_()) {
            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(1, 0));
        }

        if (hall_calibration.is_calibrated() && !was_calibrated) {
            DenseMatrix x_add = new DenseMatrix(1, 1);
            x_add.set(0, 0, hall_calibration.offset);
            observer_.plant_.x_ = (observer_.plant_.x_.add(x_add));

            profiled_goal_ = new MotionProfilePosition(observer_.plant_.x_.get(0, 0), observer_.plant_.x_.get(0, 1));
        }

        UpdateProfiledGoal(elevatorSubsystem.isOutputs_enabled_());

        DenseMatrix r = new DenseMatrix(3, 1);
        r.set(0, 0, profiled_goal_.position);
        r.set(1, 0, profiled_goal_.velocity);
        r.set(2, 0, 0);

        controller_.r_ = r;

        elevator_u = controller_.Update(observer_.plant_.x_, controller_.r_).get(0, 0); //yay!

        if (!elevatorSubsystem.isOutputs_enabled_()) {
            elevator_u = ControlsMathUtil.Cap(elevator_u, -Constants.kElevatorVoltageCap, Constants.kElevatorVoltageCap);
        } else if (!hall_calibration.is_calibrated()) {
            elevator_u = Constants.kCalibrationVoltage;
        } else if (elevatorSubsystem.encoder_fault_detected_) {
            elevator_u = 2.0;
        } else if (profiled_goal_.position <= 1e-5) {
            elevator_u = 0.0;
        }

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

        elevator_u = ControlsMathUtil.Cap(elevator_u, -Constants.kElevatorVoltageCap, Constants.kElevatorVoltageCap); //Cap... again?

        elevatorSubsystem.old_pos_ = elevatorSubsystem.getEncoder();

        DenseMatrix elevator_u_mat = new DenseMatrix(1, 1);
        elevator_u_mat.set(0, 0, elevator_u);
        observer_.Update(elevator_u_mat, y);
        plant_.Update(elevator_u_mat);

        if (elevatorSubsystem.isOutputs_enabled_()) {
            return elevator_u;
        } else {
            return 0.0;
        }
    }

}