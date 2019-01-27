package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.*;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.subsystems.controller.WristController;
import jeigen.DenseMatrix;

public class WristSubsystem extends Subsystem {
    private static WristSubsystem mInstance;
    private WristController wristController = new WristController();

    private TalonSRX mWristTalon;

    private HallCalibration hall_calibration_ = new HallCalibration(Constants.kWristHallEffectPosition);
    private boolean outputs_enabled_;

    public boolean encoder_fault_detected_;
    public double old_pos_;
    public int num_encoder_fault_ticks_ = 0;

    private double encoder_value;
    private boolean hall_status;
    private double wrist_goal_pos = 0;
    private double u = 0;

    public WristSubsystem() {
        this(false);
        wristController.SetWeights(false);
        mWristTalon = TalonSRXFactory.createDefaultTalon(Constants.kWristMotorId);
    }

    public WristSubsystem(boolean test_mode) {
        if (test_mode == false) {
            mWristTalon = TalonSRXFactory.createDefaultTalon(Constants.kWristMotorId);
        } else {
            System.out.println("WristSubsystem created in Test Mode");
        }
        zeroSensors();
    }

    public void writeToLog() {

    }

    public void fastPeriodic() {
        wristController.SetGoal(wrist_goal_pos);

        wristController.Update(this);
        u = wristController.getWrist_u();
        u = u * (1.0/Constants.kWristVoltageCap);
        mWristTalon.set(ControlMode.PercentOutput, u);
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        outputs_enabled_ = RobotState.getInstance().outputs_enabled;
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

    public static WristSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new WristSubsystem();
        }
        return mInstance;
    }

    public HallCalibration getHall_calibration_() {
        return hall_calibration_;
    }

    public boolean isOutputs_enabled_() {
        return outputs_enabled_;
    }

    public void setOutputs_enabled_(boolean outputs_enabled_) {
        this.outputs_enabled_ = outputs_enabled_;
    }

    public double getEncoder() {
        return encoder_value;
    }

    public boolean getHall() {
        return hall_status;
    }

    public void setEncoder(double encoder_value) {
        this.encoder_value = encoder_value;
    }

    public void setHall(boolean hall_status) {
        this.hall_status = hall_status;
    }

    public WristController getWristController() {
        return wristController;
    }

    public double getWrist_goal_pos() {
        return wrist_goal_pos;
    }

    public void setWrist_goal_pos(double wrist_goal_pos) {
        this.wrist_goal_pos = wrist_goal_pos;
    }

    public boolean isCalibrated() {
        return hall_calibration_.is_calibrated;
    }

}