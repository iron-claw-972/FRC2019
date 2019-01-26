package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.*;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.subsystems.controller.ElevatorController;

public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance;
    private ElevatorController elevatorController = new ElevatorController();

    private TalonSRX mElevatorTalon;

    private HallCalibration hall_calibration_ = new HallCalibration();
    private boolean outputs_enabled_;

    public boolean encoder_fault_detected_;
    public double old_pos_;
    public int num_encoder_fault_ticks_ = 0;

    private double encoder_value;
    private boolean hall_status;

    public ElevatorSubsystem() {
        this(false);
    }

    public ElevatorSubsystem(boolean test_mode) {
        if (test_mode == false) {
            mElevatorTalon = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMotorId);
        } else {
            System.out.println("ElevatorSubsystem created in Test Mode");
        }
        zeroSensors();
    }

    public void writeToLog() {

    }

    public void fastPeriodic() {
        elevatorController.Update(this);
        double u = elevatorController.getElevator_u();
        u = u * (1.0/Constants.kElevatorVoltageCap);
        mElevatorTalon.set(ControlMode.PercentOutput, u);
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

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
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

    public ElevatorController getElevatorController() {
        return elevatorController;
    }

}