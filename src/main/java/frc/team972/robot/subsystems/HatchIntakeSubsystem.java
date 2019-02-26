package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;
import frc.team972.robot.loops.ILooper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchIntakeSubsystem extends Subsystem {

    private static HatchIntakeSubsystem mInstance = new HatchIntakeSubsystem();
    private DoubleSolenoid mIntakeSolenoid;
    private boolean desiredSolenoidState = false;

    public HatchIntakeSubsystem() {
        mIntakeSolenoid = new DoubleSolenoid(Constants.kHatchIntakePistonChannelAId, Constants.kHatchIntakePistonChannelBId);
    }

    public void writeToLog() {
    }

    public void slowPeriodic() {
        if(desiredSolenoidState) { // if true, we wish to set solenoid to the open position to eject
            mIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
        } else { // if false, we want to retract the solenoid to retract our piston and prepare for another intake
            mIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
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

    public void setIntakeReady() {
        desiredSolenoidState = false;
    }

    public void setInstanceEject() {
        desiredSolenoidState = true;
    }

    public static HatchIntakeSubsystem getInstance() {
        return mInstance;
    }
}