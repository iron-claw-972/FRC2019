package frc.team972.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.team972.robot.Constants;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.util.CoordinateDriveSignal;

public class PistonClimbSubsystem extends Subsystem {
    private static PistonClimbSubsystem mInstance = new PistonClimbSubsystem(Constants.stage1Delay, Constants.stage2Delay, Constants.stage3Delay, Constants.stage4Delay, Constants.stage5Delay, Constants.stage6Delay);

    public final double HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES = Constants.HabLevelTwoElevationInches - Constants.HabLevelOneElevationInches;
    public final double GROUND_CLEARANCE_INCHES = 12; //TODO: Find actual value
    private final double ERROR_TOLERANCE = 1;
    private final double speed = 0.5;
    private final double alignmentDelay = 1.5;
    private final CoordinateDriveSignal forward = new CoordinateDriveSignal(speed, 0.0, 0.0, false);

    private Timer waitTimer = new Timer();
    private double time = 0;

    private Ultrasonic RangeSensorFront;
    private Ultrasonic RangeSensorBack;
    private double range = 0;

    private double detectionTime = 0;

    private DoubleSolenoid frontPistons;
    private DoubleSolenoid backPistons;

    private DriveSubsystem driveControl = new DriveSubsystem();

    private double[] StageClimbTimings = new double[6];
    
    private stage currentStage = stage.NOSTAGE;
    private stageState output;

    public void setDetectionTime(double detectionTime) {
        this.detectionTime = detectionTime;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setRange(double range) {
        this.range = range;
    }

    public enum stage {
        NOSTAGE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT;
    }

    public enum stageState {
        COMPLETE, IN_PROG, FAILED;
    }

    public PistonClimbSubsystem(double stage1Delay, double stage2Delay, double stage3Delay, double stage4Delay, double stage5Delay, double stage6Delay)
    {
        frontPistons = new DoubleSolenoid(1, 2);
        backPistons = new DoubleSolenoid(3, 4);
        RangeSensorFront = new Ultrasonic(1, 1);
        RangeSensorBack = new Ultrasonic(2,2);
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
        if (currentStage == stage.NOSTAGE) {
            currentStage = stage.STAGE_1;
            restartTimer();
        } else {
            currentStage = stage.ABORT;
        }
    }

    public void takeTime() {
        setTime(waitTimer.get());
    }

    public void takeRange() {
        if (currentStage == stage.STAGE_2)
        {
            setRange(RangeSensorFront.getRangeInches());
        }
        else if (currentStage == stage.STAGE_5)
        {
            setRange(RangeSensorBack.getRangeInches());
        }
    }

    public void slowPeriodic() {

    }

    public void fastPeriodic() {

        if (!(currentStage == stage.NOSTAGE)) {
            takeTime();
            takeRange();
        }

        switch (currentStage) {
            case STAGE_1:
                output = climbStage1(StageClimbTimings[0]);
                if (output == stageState.COMPLETE) {
                    restartTimer();
                    currentStage = stage.STAGE_2;
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_2:
                output = climbStage2(StageClimbTimings[1], true);
                if (output == stageState.COMPLETE) {
                    detectionTime = 0;
                    restartTimer();
                    currentStage = stage.STAGE_3;
                } else if (output == stageState.FAILED){
                    detectionTime = 0;
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_3:
                output = climbStage3(StageClimbTimings[2]);
                if (output == stageState.COMPLETE) {
                    restartTimer();
                    currentStage = stage.STAGE_4;
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_4:
                output = climbStage4(StageClimbTimings[3]);
                if (output == stageState.COMPLETE) {
                    restartTimer();
                    currentStage = stage.STAGE_5;
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_5:
                output = climbStage5(StageClimbTimings[4], true);
                if (output == stageState.COMPLETE) {
                    detectionTime = 0;
                    currentStage = stage.STAGE_6;
                    restartTimer();
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_6:
                output = climbStage6(StageClimbTimings[5]);
                if (output == stageState.COMPLETE) {
                    currentStage = stage.NOSTAGE;
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case ABORT:
                abortClimb();
                currentStage = stage.NOSTAGE;
                break;
            default:
                break;
        }
    }

    public stageState climbStage1(double waitTime)// raising the front pistons
    {
        // restart the timer so that it is representing the time spent on this stage
        setFrontPistonsState(true);
        if(time < waitTime) {
            return stageState.IN_PROG;
        } else {
            return stageState.COMPLETE;
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
                return stageState.COMPLETE;
            } else {
                return stageState.IN_PROG;
            }

        } else if(time >= waitTime && safety == true && detectionTime == 0)
        {// abort the climbing if it was trying for more than 7.5 seconds
            // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
            return stageState.FAILED;
        } else
        {
            detectionTime = 0;
            return stageState.IN_PROG;
        }

    }

    public stageState climbStage3(double waitTime) // retract the front pistons to lower the front wheels, over the platform
    {
        setFrontPistonsState(false);
        if (time < waitTime) {
            return stageState.COMPLETE;
        } else {
            return stageState.IN_PROG;
        }
    }

    public stageState climbStage4(double waitTime) // raise the back pistons
    {
        setBackPistonsState(true);
        if (time < waitTime) {
            return stageState.COMPLETE;
        } else {
            return stageState.IN_PROG;
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
                return stageState.COMPLETE;
            } else {
                detectionTime = 0;
                return stageState.IN_PROG;
            }

        } else if(time >= waitTime && safety == true && detectionTime == 0)
        {// abort the climbing if it was trying for more than 7.5 seconds
            // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
            return stageState.FAILED;
        } else
        {
            return stageState.IN_PROG;
        }

    }

    public stageState climbStage6(double waitTime) // retract the back pistons to close the climbing loop
    {
        // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(false);
        if(time <= waitTime) {
            return stageState.IN_PROG;
        } else {
            return stageState.COMPLETE;
        }
    }

    public void abortClimb() {
        driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0, 0, 0, false));
        setFrontPistonsState(false);
        setBackPistonsState(false);
        waitTimer.stop();
        time = 0;
        waitTimer.reset();
    }

    public void setFrontPistonsState(boolean value)
    {
        if(value)
        {
            frontPistons.set(DoubleSolenoid.Value.kForward);
        }
        else if(!value)
        {
            frontPistons.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void setBackPistonsState(boolean value)
    {
        if(value)
        {
            backPistons.set(DoubleSolenoid.Value.kForward);
        }
        else if(!value)
        {
            backPistons.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public boolean checkSystem() {
        return false;
    }

    public void outputTelemetry() {

    }

    public void stop() {
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public static PistonClimbSubsystem getInstance() {
        return mInstance;
    }
}
