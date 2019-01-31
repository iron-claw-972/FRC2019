package frc.team972.robot.subsystems;

import frc.team972.robot.loops.ILooper;

public class PistonClimb extends Subsystem {
    private static PistonClimb mInstance = new PistonClimb();

    private final double HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES = Constants.HabLevelTwoElevationInches - Constants.HabLevelOneElevationInches;
    private final double GROUND_CLEARANCE_INCHES = 12; //TODO: Find actual value
    private final double ERROR_TOLERANCE = 1;
    private final double speed = 0.5;
    private final double alignmentDelay = 1.5;
    private final CoordinateDriveSignal forward = new cooCdinateDriveSignal(speed, 0.0, 0.0, false);

    private Timer waitTimer = new Timer();
    private double time = 0;

    private Ultrasonic RangeSensorFront;
    private Ultrasonic RangeSensorBack;
    private double range = 0;

    private double detectionTime = 0;

    private DoubleSolenoid frontPistons;
    private DoubleSolenoid backPistons;

    private static Piston_Climb mInstance = new Piston_Climb();
    private Drive driveControl = new Drive();

    public double[] StageClimbTimings = new double[6];
    private boolean testing;

    public enum stage {
        NOSTAGE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT;
    }

    public enum stageState {
        COMPLETE, IN_PROGRESS, FAILED;
    }

    private stage currentStage = NOSTAGE;
    private stageState pistonClimbOutput;

    public void PistonClimb(double stage1Delay, double stage2Delay, double stage3Delay, double stage4Delay, double stage5Delay, double stage6Delay, boolean _testing)
    {
        frontPistons = new DoubleSolenoid(1, 2);
        backPistons = new DoubleSolenoid(3, 4);
        RangeSensorFront = new UltraSonic(1, 1);
        RangeSensorBack = new UltraSonic(2,2);
        StageClimbTimings[0] = stage1Delay;
        StageClimbTimings[1] = stage2Delay;
        StageClimbTimings[2] = stage3Delay;
        StageClimbTimings[3] = stage4Delay;
        StageClimbTimings[4] = stage5Delay;
        StageClimbTimings[5] = stage6Delay;
        testing = _testing;
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
            restartTimer();
        } else {
            currentStage = ABORT;
        }
    }

    public void takeTime() {
        time = waitTimer.get();
    }

    public void takeRange() {
        if (testing)
        {
            pistonClimbTest.testPistonClimb(time, currentStage);
        }
        else if (currentStage == STAGE_2)
        {
            range = RangeSensorFront.getRangeInches();
        }
        else if (currentStage == STAGE_5)
        {
            range = RangeSensorBack.getRangeInches();
        }
    }

    public void fastPeriodic() {

        takeTime();
        takeRange();

        switch (currentStage) {
        case STAGE_1:
            output = climbStage1(StageClimbTimings[0]);
            if (output == COMPLETE) {
                restartTimer();
                currentStage = STAGE_2;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_2:
            stageState output = climbStage2(StageClimbTimings[1], true);
            if (output == COMPLETE) {
                detectionTime = 0;
                restartTimer();
                currentStage = STAGE_3;
            } else if (output == FAILED){
                detectionTime = 0;
                currentStage = ABORT;
            }
            break;
        case STAGE_3:
            output = climbStage3(StageClimbTimings[2]);
            if (output == COMPLETE) {
                restartTimer();
                currentStage = STAGE_4;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_4:
            output = climbStage4(StageClimbTimings[3]);
            if (output == COMPLETE) {
                restartTimer();
                currentStage = STAGE_5;
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_5:
            output = climbStage5(StageClimbTimings[4], true);
            if (output == COMPLETE) {
                detectionTime = 0;
                currentStage = STAGE_6;
                restartTimer();
            } else if (output == FAILED){
                currentStage = ABORT;
            }
            break;
        case STAGE_6:
            output = climbStage6(StageClimbTimings[5]);
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
        if(time < waitTime) {
            return IN_PROG;
        } else {
            return COMPLETE;
        }
    }

    public stageState climbStage2(double waitTime, boolean safety) // safety is recommended while the numbers have not been calibrated as desired
    { // move forward until the stage is detected and waitTime driven forward since detection

        driveControl.setOpenLoopMecanum(forward); // drive forward
        if(range <= (HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES - ERROR_TOLERANCE)) // check for UltraSonic sensor to be over the stage platform
        {
            if (detectionTime == 0) {
                detectionTime = waitTimer.get();
            }

            if(time >= detectionTime + alignmentDelay && time >= waitTime) // drive forward for waitTime and then stop
            {
                driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false)); // stop after "waitTime"
                return COMPLETE;
            } else {
                return IN_PROGRESS;
            }

        } else if(time >= waitTime && safety == true && detectionTime == 0)
        {// abort the climbing if it was trying for more than 7.5 seconds
            // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
            return FAILED;
        } else
        {
            return IN_PROGRESS;
        }

    }

    public stageState climbStage3(double waitTime) // retract the front pistons to lower the front wheels, over the platform
    {
        setFrontPistonsState(false);
        if (time < waitTime) {
            return COMPLETE;
        } else {
            return IN_PROGRESS;
        }
    }

    public stageState climbStage4(double waitTime) // raise the back pistons
    {
        setBackPistonsState(true);
        if (time < waitTime) {
            return COMPLETE;
        } else {
            return IN_PROGRESS;
        }
    }

    public stageState climbStage5(double waitTime, boolean safety) // be careful when using the waitTime on this
    { // move forward until the stage is detected and waitTime driven forward since detection

        driveControl.setOpenLoopMecanum(forward); // drive forward
        if(range <= (GROUND_CLEARANCE_INCHES + ERROR_TOLERANCE)) // check for UltraSonic sensor to be over the stage platform
        {
            if (detectionTime == 0) {
                detectionTime = waitTimer.get();
            }

            if(time >= detectionTime + alignmentDelay && time >= waitTime) // drive forward for waitTime and then stop
            {
                driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false)); // stop after "waitTime"
                return COMPLETE;
            } else {
                return IN_PROGRESS;
            }

        } else if(time >= waitTime && safety == true && detectionTime == 0)
        {// abort the climbing if it was trying for more than 7.5 seconds
            // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
            return FAILED;
        } else
        {
            return IN_PROGRESS;
        }

    }

    public stageState climbStage6(double waitTime) // retract the back pistons to close the climbing loop
    {
        // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(false);
        if(time <= waitTime) {
            return IN_PROGRESS;
        } else {
            return COMPLETE;
        }
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