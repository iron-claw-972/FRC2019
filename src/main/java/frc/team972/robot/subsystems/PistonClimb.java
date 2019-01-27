package frc.team972.robot.subsystems;

import frc.team972.robot.loops.ILooper;

public class PistonClimb extends Subsystem {
    private static PistonClimb mInstance = new ExampleSubsystem();

    private final double HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES = Constants.HabLevelTwoElevationInches - Constants.HabLevelOneElevationInches;
    private final double speed = 0.5;
    private final double alignmentDelay = 1.5;
    private final coordinateDriveSignal forward = new CoordinateDriveSignal(speed, 0.0, 0.0, false);

    private boolean takingTime = false;
    private Timer waitTimer = new Timer();
    private double time = 0;

    private boolean takingRange = false;
    private Ultrasonic RangeSensor = new Ultrasonic(1, 1);
    private double range = 0;

    private DoubleSolenoid frontPistons;
    private DoubleSolenoid backPistons;

    private static Piston_Climb mInstance = new Piston_Climb();
    private Drive driveControl = new Drive();

    private boolean isClimbing = false;

    public double[] StageClimbTimings = new double[6];

    private stage currentStage = NOSTAGE;

    public enum stage {
        NOSTAGE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT, END;
    }

    public enum stageState {
        COMPLETE, IN_PROGRESS, FAILED;
    }

    public void PistonClimb(double stage1Delay, double stage2Delay, double stage3Delay, double stage4Delay, double stage5Delay, double stage6Delay)
    {
        frontPistons = new DoubleSolenoid(1, 2);
        backPistons = new DoubleSolenoid(3, 4);
        StageClimbTimings[0] = stage1Delay;
        StageClimbTimings[1] = stage2Delay;
        StageClimbTimings[2] = stage3Delay;
        StageClimbTimings[3] = stage4Delay;
        StageClimbTimings[4] = stage5Delay;
        StageClimbTimings[5] = stage6Delay;
    }

    private void restartTimer() {
        waitTimer.stop();
        time = 0;
        waitTimer.reset();
        waitTimer.start();
    }

    public void writeToLog() {
    }

    public void beginClimb() {
        if (currentStage == NOSTAGE) {
            currentStage = STAGE_1;
        } else {
            currenStage = ABORT;
        }
    }

    public void fastPeriodic() {
        switch (currentStage) {
        case STAGE_1:
            stageState output = climbStage1(StageClimbTimings[0]);
            if (output == COMPLETE) {
                currentStage = STAGE_2;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_2:
            stageState output = climbStage2(StageClimbTimings[1]);
            if (output == COMPLETE) {
                currentStage = STAGE_3;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_3:
            stageState output = climbStage3(StageClimbTimings[2]);
            if (output == COMPLETE) {
                currentStage = STAGE_4;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_4:
            stageState output = climbStage4(StageClimbTimings[3]);
            if (output == COMPLETE) {
                currentStage = STAGE_5;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_5:
            stageState output = climbStage5(StageClimbTimings[4]);
            if (output == COMPLETE) {
                currentStage = STAGE_6;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_6:
            stageState output = climbStage6(StageClimbTimings[5]);
            if (output == COMPLETE) {
                currentStage = NOSTAGE;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case ABORT:
            abortClimb();
            currentStage = NOSTAGE;
            break;
        }
    }

    public stageState climbStage1(double waitTime)// raising the front pistons
    {
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        setFrontPistonsState(true);
        while(time <= waitTime);
        takingTime = false;
        return true;
    }

    public stageState climbStage2(double waitTime, boolean safety) // safety is recommended while the numbers have not been calibrated as desired
    { // move forward until the stage is detected and waitTime driven forward since detection
        restartTimer();
        driveControl.setOpenLoopMecanum(forward); // drive forward
        double detectionTime = 0;
        boolean abortion = false;
        if(range <= HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES) // check for UltraSonic sensor to be over the stage platform
        {

                detectionTime = waitTimer.get();
                if(time >= detectionTime + alignmentDelay && time >= waitTime) // drive forward for waitTime and then stop
                {
                    driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false)); // stop after "waitTime"
                    return COMPLETE;
                }
        } else if(time >= waitTime && safety == true && detectionTime == 0)
        {// abort the climbing if it was trying for more than 7.5 seconds
            // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
            abortion = true;
        }
        if(abortion) // check if the stage has aborted or not
        {
            return FAILED;
        }
        else
        {
            return IN_PROGRESS;
        }
    }

    public stageState climbStage3(double waitTime) // retract the front pistons to lower the front wheels, over the platform
    {
        restartTimer();
        setFrontPistonsState(false);
        while(time <= waitTime);
        return true;
    }

    public stageState climbStage4(double waitTime) // raise the back pistons
    {
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(true);
        while(time <= waitTime);
        return true;
    }

    public stageState climbStage5(double waitTime) // be careful when using the waitTime on this
    { // move forward until the stage is detected and waitTime driven forward since detection
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        CoordinateDriveSignal forward = new CoordinateDriveSignal(speed, 0.0, 0.0, false);
        driveControl.setOpenLoopMecanum(forward); // drive forward
        while(time <= waitTime);
        return true;
    }

    public stageState climbStage6(double waitTime) // retract the back pistons to close the climbing loop
    {
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(false);
        while(time <= waitTime);
        return true;
    }

    public void abortClimb() {
        driveControl.mecanumDriveSignalDesired(new coordinateDriveSignal(0, 0, 0, false));
        frontPistonsState(false);
        backPistonsState(false);
        waitTimer.stop();
        time = 0;
        waitTimer.reset();
    }

    public void setFrontPistonsState(boolean value)
    {
        if(value == true)
        {
            frontPistons.set(DoubleSolenoid.Value.kForward);
        }
        else if(value == false)
        {
            frontPistons.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void setBackPistonsState(boolean value)
    {
        if(value == true)
        {
            backPistons.set(DoubleSolenoid.Value.kForward);
        }
        else if(value == false)
        {
            backPistons.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean checkSystem() {

    }

    public void outputTelemetry() {

    }

    public void stop() {
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public static ExampleSubsystem getInstance() {
        return mInstance;
    }
}