package frc.team972.robot.subsystems;

import frc.team972.robot.loops.ILooper;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class HatchIntake extends Subsystem {

    public boolean hatchIntakeWorks = false;

    private static HatchIntake mHatchIntake = new HatchIntake();

    public DoubleSolenoid hatchIntakeDoubleSolenoid;

    public void init() {
        hatchIntakeDoubleSolenoid = new DoubleSolenoid(-1, -1);
    }

    public void open() {
        hatchIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void close() {
        hatchIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop() {
        hatchIntakeDoubleSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void toggle() {
        if (hatchIntakeWorks){
            open();
        }
        else{
            close();
        }
    }

    public void writeToLog() {
        //Doesn't do stuff
    }

    public void readPeriodicInputs() {
        //Doesn't do stuff
    }

    public void writePeriodicOutputs() {
        //System.out.println("Test");
        //Doesn't do stuff
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        //Doesn't do stuff
    }



    public void zeroSensors() {
        //Doesn't do stuff
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        //Doesn't do stuff
    }

    public static HatchIntake getInstance() {
        return mHatchIntake;
    }
}