package frc.team972.robot.subsystems;

public class SuperstructureSubsystem extends Subsystem {

    WristSubsystem wrist_ = WristSubsystem.getInstance();
    ElevatorSubsystem elevator_ = ElevatorSubsystem.getInstance();

    private double wristUserGoal_ = 0;
    private double elevatorUserGoal_ = 0;

    private double kSafetyMargin = 2.0; //degrees

    private static SuperstructureSubsystem mInstance = new SuperstructureSubsystem();

    @Override
    public void fastPeriodic() {
        //lol redesign made my job 99% easier
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    public boolean checkTol(double a, double b, double tol) {
        if (Math.abs(a - b) <= tol) {
            return true;
        } else {
            return false;
        }
    }

    public static SuperstructureSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SuperstructureSubsystem();
        }
        return mInstance;
    }

    public boolean checkBounds(double num, double low, double high) {
        if ((num >= low) && (num <= high)) {
            return true;
        } else {
            return false;
        }
    }

}