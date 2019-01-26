package frc.team972.robot.subsystems;

import frc.team972.robot.loops.ILooper;

public abstract class Subsystem {
    public void writeToLog() {
    }

    public void fastPeriodic() {
    }

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }
}