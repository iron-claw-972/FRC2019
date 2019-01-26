package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team972.robot.Constants;
import frc.team972.robot.controls.*;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.subsystems.controller.ElevatorController;

public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance = new ElevatorSubsystem();
    private ElevatorController elevatorController = new ElevatorController();

    private TalonSRX mElevatorTalon;

    private HallCalibration hall_calibration_ = new HallCalibration();
    private boolean outputs_enabled_;

    public boolean encoder_fault_detected_;
    public double old_pos_;
    public int num_encoder_fault_ticks_ = 0;

    private int encoder_ticks;
    private boolean hall_status;

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

    public HallCalibration getHall_calibration_() {
        return hall_calibration_;
    }

    public boolean isOutputs_enabled_() {
        return outputs_enabled_;
    }

    public double getEncoder() {
        return encoder_ticks;
    }

    public boolean getHall() {
        return hall_status;
    }

    public void setEncoder(int encoder_ticks) {
        this.encoder_ticks = encoder_ticks;
    }

    public void setHall(boolean hall_status) {
        this.hall_status = hall_status;
    }

    public ElevatorController getElevatorController() {
        return elevatorController;
    }

}