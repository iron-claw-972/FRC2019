package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;
import jeigen.DenseMatrix;

import java.util.Set;

public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance = new ElevatorSubsystem();

    private TalonSRX mElevatorTalon;

    private HallCalibration hall_calibration = new HallCalibration();
    private boolean outputs_enabled;

    public ElevatorSubsystem() {
        mElevatorTalon = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMotorId);
        zeroSensors();
    }

    public void writeToLog() {

    }

    public void fastPeriodic() {
        mElevatorTalon.set(ControlMode.PercentOutput, 0);
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
    }

    public void stop() {
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    private double handleDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public static ElevatorSubsystem getInstance() {
        return mInstance;
    }

    public HallCalibration getHall_calibration() {
        return hall_calibration;
    }

    public boolean isOutputs_enabled() {
        return outputs_enabled;
    }

    public double getEncoder() {
        return 0.0; //TODO: Implement
    }

    public boolean getHall() {
        return false; //TODO: Implement
    }
}

class ElevatorController {
    public StateSpacePlant plant_ = new StateSpacePlant(1, 3, 1);
    public StateSpaceController controller_ = new StateSpaceController(1, 3, 1);
    public StateSpaceObserver observer_ = new StateSpaceObserver(1, 3, 1);

    MotionProfilePosition unprofiled_goal_;
    MotionProfilePosition profiled_goal_;

    public void SetWeights(boolean second_stage) {
        //TODO: Switch out controller gains
    }

    public MotionProfilePosition UpdateProfiledGoal(boolean outputs_enabled) {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(Constants.kElevatorConstraints, unprofiled_goal_, profiled_goal_);

        if(outputs_enabled) {
            profiled_goal_ = profile.Calculate(5.0 * 0.001); // 5 ms
        } else {
            profiled_goal_ = profile.Calculate(0.0); //deded robot
        }

        return profiled_goal_;
    }

    public void Update(ElevatorInputProto input, ElevatorSubsystem elevatorSubsystem) {
        HallCalibration hall_calibration = elevatorSubsystem.getHall_calibration();
        boolean was_calibrated = hall_calibration.is_calibrated();

        DenseMatrix y = new DenseMatrix(1, 1);
        y.set(1, 1, hall_calibration.Update(elevatorSubsystem.getEncoder(), elevatorSubsystem.getHall()));

        if (elevatorSubsystem.getHall_calibration().is_calibrated) {
            SetWeights(observer_.x_.get(0, 0) >= 1.0);
        } else {
            SetWeights(false);
        }

        if (!elevatorSubsystem.isOutputs_enabled()) {
            profiled_goal_ = new MotionProfilePosition(observer_.x_.get(0,0), observer_.x_.get(1, 0));
        }

        if(hall_calibration.is_calibrated() && !was_calibrated) {
            DenseMatrix x_add = new DenseMatrix(1,1);
            x_add.set(1,1, hall_calibration.offset);
            observer_.x_ = (observer_.x_.add(x_add));

            profiled_goal_ = new MotionProfilePosition(observer_.x_.get(0,0), observer_.x_.get(0, 1));
        }

        UpdateProfiledGoal(elevatorSubsystem.isOutputs_enabled());

        controller_.r()


    }

}

class ElevatorInputProto {
    double elevator_encoder;
    boolean elevator_hall;

}

class HallCalibration {
    public boolean is_calibrated = false;
    public double offset = 0;

    public boolean is_calibrated() {
        return is_calibrated;
    }

    //TODO: Implement
    public double Update(double main_sensor_value, boolean hall_value) {
        return 0.0;
    }
}