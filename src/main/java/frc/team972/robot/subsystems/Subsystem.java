package frc.team972.robot.subsystems;

public abstract class Subsystem {
    public void writeToLog() {
    }

    public void slowPeriodic() {
    }

    public void fastPeriodic() {
    }

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors() {
    }
}