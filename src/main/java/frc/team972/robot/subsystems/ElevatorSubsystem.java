package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.*;
import frc.team972.robot.Constants;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.loops.ILooper;

public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance = new ElevatorSubsystem();

    private TalonSRX mElevatorTalon;

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

}
