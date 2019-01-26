package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import frc.team972.robot.Constants;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;



public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance = new ElevatorSubsystem();

    private static final double maxSpeed = 1; //Feet per Second;

    private TalonSRX mElevatorTalon;
    private Encoder mElevatorEncoder;

    private boolean autoMoving;

    private double axisValue;

    private double voltage;

    public ElevatorSubsystem() {
        mElevatorTalon = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMotorId);
        mElevatorEncoder = new Encoder(-1, -2); //TODO Find channels
        autoMoving = false;
        zeroSensors();
        voltage = 0;
    }

    public void writeToLog() {
        if (autoMoving) {
            System.out.println("Automatically moving. Voltage: " + voltage);
        } else {
            System.out.println("Manually Moving. Voltage: " + voltage);
        }
    }

    public void readPeriodicInputs() {
    }

    public void readControllerInputs(double axisValue) {
        this.axisValue = axisValue;
    }

    public void writePeriodicOutputs() {
        if(autoMoving) {
            voltage = 0.0;
        } else {
            voltage = handleDeadband(axisValue * 0.2, 0.05);
        }

        mElevatorTalon.set(ControlMode.PercentOutput, voltage);
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
    }

    public void stop() {
    }

    public void zeroSensors() {
        mElevatorEncoder.reset();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    private double handleDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public static ElevatorSubsystem getInstance() {
        return mInstance;
    }

}
