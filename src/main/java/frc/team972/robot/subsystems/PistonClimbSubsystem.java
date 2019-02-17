package frc.team972.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.team972.robot.Constants;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.util.CoordinateDriveSignal;

public class PistonClimbSubsystem extends Subsystem {
    private static PistonClimbSubsystem mInstance = new PistonClimbSubsystem();

    public final double HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES = Constants.HabLevelTwoElevationInches - Constants.HabLevelOneElevationInches;
    public final double GROUND_CLEARANCE_INCHES = 12; //TODO: Find actual value
    private final double ERROR_TOLERANCE = 1;
    private final double ALIGN_DELAY = 1.5;
    private final double G_STOP_THRESHOLD = 1.5; //TODO: figure out the actual speed of the robot and therefore get max deceleration
    private final CoordinateDriveSignal forward = new CoordinateDriveSignal(0.5, 0.0, 0.0, false);

    private double startTime = 0;
    private double waitTimer = 0;
    private double detectionTime = 0;
    private double time = 0;

    private AHRS ahrs = new AHRS(SPI.Port.kMXP, (byte) 200); //TODO: Configure the actual ahrs port
    private DoubleSolenoid frontPistons;
    private DoubleSolenoid backPistons;
    private double acceleration = 0;

    private DriveSubsystem driveControl = new DriveSubsystem();
    //TODO: Fix DriveSubsystem problems/affirm functionality

    private double[] StageClimbTimings = new double[6];
    private stage currentStage = stage.NOSTAGE;
    private stageState output;

    private boolean notTesting = false;

    private boolean manual = false;

    public void setAcceleration(double acceleration) 
    {
    	this.acceleration = acceleration;
    }
    
    public void setPistonClimbNotTesting(boolean notTesting) {
        this.notTesting = notTesting;
    }

    public void setDetectionTime(double detectionTime) {
        this.detectionTime = detectionTime;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public enum stage {
        NOSTAGE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT;
    }

    public enum stageState {
        COMPLETE, IN_PROG, FAILED;
    }

    public PistonClimbSubsystem()
    { //TODO: Fix problem with initializer (ExceptionInitializerError)
    	System.out.println("1af");
        frontPistons = new DoubleSolenoid(0, 1);
        System.out.println("2af");
        backPistons = new DoubleSolenoid(2, 3);
        StageClimbTimings[0] = Constants.stage1Delay;
        StageClimbTimings[1] = Constants.stage2Delay;
        StageClimbTimings[2] = Constants.stage3Delay;
        StageClimbTimings[3] = Constants.stage4Delay;
        StageClimbTimings[4] = Constants.stage5Delay;
        StageClimbTimings[5] = Constants.stage6Delay;
    }

    private void restartTimer() {
        waitTimer = 0;
        time = 0;
        startTime = System.currentTimeMillis();
    }

    public void writeToLog() {
    }

    public void beginClimb()
    {
        if (currentStage == stage.NOSTAGE) {
            currentStage = stage.STAGE_1;
            restartTimer();
        } else {
            manual = false;
            currentStage = stage.ABORT;
        }
    }

    public void switchMode()
    {
        if(manual == true)
        {
            currentStage = stage.ABORT;
            manual = false;
        }
        else
        {
            manual = true;
        }
    }

    public void takeTime() {
        waitTimer = System.currentTimeMillis() - startTime;
        setTime(waitTimer);
    }

    public void takeAcceleration() {
        if (currentStage == stage.STAGE_2)
        {
            setAcceleration(Math.sqrt(Math.pow(ahrs.getWorldLinearAccelX(), 2) + Math.pow(ahrs.getWorldLinearAccelY(), 2)));
        }
        else if (currentStage == stage.STAGE_5)
        {
        	setAcceleration(Math.sqrt(Math.pow(ahrs.getWorldLinearAccelX(), 2) + Math.pow(ahrs.getWorldLinearAccelY(), 2)));
        }
    }

    public void slowPeriodic() {

    }

    public void frontPistonsManual()
    {
        if ((frontPistons.get() == DoubleSolenoid.Value.kReverse) && (manual)) {
            currentStage = stage.STAGE_1;
        } else {
            currentStage = stage.STAGE_3;
        }
    }

    public void backPistonsManual()
    {
        if ((backPistons.get() == DoubleSolenoid.Value.kReverse) && (manual)) {
            currentStage = stage.STAGE_4;
        } else {
            currentStage = stage.STAGE_6;
        }
    }

    public void fastPeriodic() {//Checks stage and completion requirements

        if (!(currentStage == stage.NOSTAGE)) {
            takeTime();
            takeAcceleration();
        }

        switch (currentStage) {
            case STAGE_1:
                output = climbStage1(StageClimbTimings[0]);
                if (output == stageState.COMPLETE) {
                    if(manual == false)
                    {
                        currentStage = stage.STAGE_2;
                    } else {
                        currentStage = stage.NOSTAGE;
                    }
                    restartTimer();
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_2:
                output = climbStage2(StageClimbTimings[1], true);
                if (output == stageState.COMPLETE) {
                    detectionTime = 0;
                    currentStage = stage.STAGE_3;
                    restartTimer();
                } else if (output == stageState.FAILED){
                    detectionTime = 0;
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_3:
                output = climbStage3(StageClimbTimings[2]);
                if (output == stageState.COMPLETE) {
                    if(manual == false)
                    {
                        currentStage = stage.STAGE_4;
                    } else {
                        currentStage = stage.NOSTAGE;
                    }
                    restartTimer();
                } else if (output == stageState.FAILED){
                    currentStage = stage.ABORT;
                }
                break;
            case STAGE_4:
                output = climbStage4(StageClimbTimings[3]);
                if (output == stageState.COMPLETE) {
                    if(manual == false)
                    {
                        currentStage = stage.STAGE_5;
                    } else {
                        currentStage = stage.NOSTAGE;
                    }
                    restartTimer();
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
                    if(manual == false) {
                        currentStage = stage.ABORT;
                    } else {
                        currentStage = stage.NOSTAGE;
                    }
                    restartTimer();
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

    public stageState climbStage1(double waitTime)//extends front pistons; moves onto next stage when certain time is reached
    {
    	System.out.println("climbStage1 check1");

        if (notTesting) {
        	System.out.println("notTesting " + notTesting);
            setFrontPistonsState(true);
        }

        System.out.println("climbstage1 check2");

        if(time < waitTime) {
        	System.out.println("climbstage1 returning IN_PROG");
            return stageState.IN_PROG;
        } else {
        	System.out.println("climbstage1 returning COMPLETE");
            return stageState.COMPLETE;
        }
    }

    public stageState climbStage2(double waitTime, boolean safety) // safety is recommended while the numbers have not been calibrated as desired
    { // move forward until the stage is detected and waitTime driven forward since detection

        driveControl.setOpenLoopMecanum(forward); // drive forward
        if(acceleration <= G_STOP_THRESHOLD) // check for continued impact with wall
        {
            if (detectionTime == 0) {
                detectionTime = System.currentTimeMillis() - startTime;
            }

            if(time >= detectionTime + ALIGN_DELAY && time >= waitTime) // drive forward for waitTime and then stop
            {
                driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0, 0, 0, false));
                // stop after "waitTime"
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

    public stageState climbStage3(double waitTime) //Retracts the front pistons; moves on when time is reached
    {
    	System.out.println("climbStage2 check1");

        if (notTesting) {
        	System.out.println("notTesting " + notTesting);
            setFrontPistonsState(false);
        }

        System.out.println("climbstage2 check2");

        if(time < waitTime) {
        	System.out.println("climbstage2 returning IN_PROG");
            return stageState.IN_PROG;
        } else {
        	System.out.println("climbstage2 returning COMPLETE");
            return stageState.COMPLETE;
        }
    }

    public stageState climbStage4(double waitTime) //Extends the back pistons; moves on when time is reached
    {
    	System.out.println("climbStage4 check1");

        if (notTesting) {
        	System.out.println("notTesting " + notTesting);
            setBackPistonsState(true);
        }

        System.out.println("climbstage4 check2");

        if(time < waitTime) {
        	System.out.println("climbstage4 returning IN_PROG");
            return stageState.IN_PROG;
        } else {
        	System.out.println("climbstage4 returning COMPLETE");
            return stageState.COMPLETE;
        }
    }

    public stageState climbStage5(double waitTime, boolean safety) // be careful when using the waitTime on this
    { // move forward until the stage is detected and waitTime driven forward since detection

        driveControl.setOpenLoopMecanum(forward); // drive forward
        if(acceleration <= G_STOP_THRESHOLD) // check for continued contact with wall
        {
            if (detectionTime == 0) {
                detectionTime = System.currentTimeMillis() - startTime;
            }

            if(time >= detectionTime + ALIGN_DELAY && time >= waitTime) // drive forward for waitTime and then stop
            {
                driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false));
                // stop after "waitTime"
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

    public stageState climbStage6(double waitTime) //retract the back pistons; end the climb after a certain period of time
    {
    	System.out.println("climbStage6 check1");

        if (notTesting) {
        	System.out.println("notTesting " + notTesting);
            setBackPistonsState(false);
        }

        System.out.println("climbstage6 check2");

        if(time < waitTime) {
        	System.out.println("climbstage6 returning IN_PROG");
            return stageState.IN_PROG;
        } else {
        	System.out.println("climbstage6 returning COMPLETE");
            return stageState.COMPLETE;
        }
    }

    public void abortClimb() {//resets variables and puts robot into starting state (still, no pistons out)
        driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0, 0, 0, false));
        acceleration = 0;
        setFrontPistonsState(false);
        setBackPistonsState(false);
        waitTimer = 0;
        startTime = 0;
        detectionTime = 0;
        time = 0;
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
