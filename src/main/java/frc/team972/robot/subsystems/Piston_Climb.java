package frc.team972.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.Constants;

public class Piston_Climb extends Subsystem
{
    private final double HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES = Constants.HabLevelTwoElevationInches - Constants.HabLevelOneElevationInches;
    private final double speed = 0.5;
    private final double alignmentDelay = 1.5;
    
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
    
    public enum stage {
    	STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT, END;
    }
    
    public void setStageClimbTimings(double stage1Delay, double stage2Delay, double stage3Delay, double stage4Delay, double stage5Delay, double stage6Delay)
    { // use this in the robot code to state the climb timings, you can read how each stage uses these timings in each respective function
        StageClimbTimings[0] = stage1Delay;
        StageClimbTimings[1] = stage2Delay;
        StageClimbTimings[2] = stage3Delay;
        StageClimbTimings[3] = stage4Delay;
        StageClimbTimings[4] = stage5Delay;
        StageClimbTimings[5] = stage6Delay;
    }

    public void PistonClimb()
    {
        frontPistons = new DoubleSolenoid(1, 2);
        backPistons = new DoubleSolenoid(3, 4);
    }

    private void restartTimer() {
        waitTimer.stop();
        time = 0;
        waitTimer.reset();
        waitTimer.start();
    }

    public void startClimbing() {
        ClimbingManager(STAGE_1);
    }

    private void ClimbingManager(stage currentStage)
    {
        switch (currentStage) {
			case STAGE_1:
				if (climbStage1(StageClimbTimings[0]) == true) {
					ClimbingManager(true, STAGE_2);
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
					break;
			case STAGE_2:
				if (climbStage2(StageClimbTimings[1], true) == true) {
					ClimbingManager(true, STAGE_3);
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
				break;
			case STAGE_3:
				if (climbStage3(StageClimbTimings[2]) == true) {
					ClimbingManager(true, STAGE_4);
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
				break;
			case STAGE_4:
				if (climbStage4(StageClimbTimings[3]) == true) {
					ClimbingManager(true, STAGE_5);
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
				break;
			case STAGE_5:
				if (climbStage5(StageClimbTimings[4]) == true) {
					ClimbingManager(true, STAGE_6);
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
				break;
			case STAGE_6:
				if (climbStage6(StageClimbTimings[5]) == true) {
					ClimbingManager(false, END);
					//TODO: Output "done" to driver station
				} else {
					ClimbingManager(false, ABORT);
					//This should never occur, but if it does, there's an error
				}
				break;
			case ABORT:
			    abortClimb();
			    ClimbingManager(true, END);
				break;
			case END:
                break;
			}
		}
    	/*
        isClimbing = true;
        waitTimer.start();
        if(climbStage1(StageClimbTimings[0]) == true)
        {
            if(climbStage2(StageClimbTimings[1], true) == true)
            {
                if(climbStage3(StageClimbTimings[2]) == true)
                {
                    if(climbStage4(StageClimbTimings[3]) == true)
                    {
                        if(climbStage5(StageClimbTimings[4]) == true)
                        {
                            if(climbStage6(StageClimbTimings[5]) == true)
                            {
                                isClimbing = false;
                                // TODO: OUTPUT TO THE FRC DRIVER STATION CONSOLE: CLIMB COMPLETE
                            }
                        }
                    }
                }
            }
        }
        */

    public boolean climbStage1(double waitTime)// raising the front pistons
    {
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        setFrontPistonsState(true);
        while(time <= waitTime);
        takingTime = false;
        return true;
    }

    public boolean climbStage2(double waitTime, boolean safety) // safety is recommended while the numbers have not been calibrated as desired
    { // move forward until the stage is detected and waitTime driven forward since detection
        restartTimer();
        CoordinateDriveSignal forward = new CoordinateDriveSignal(speed, 0.0, 0.0, false);
        driveControl.setOpenLoopMecanum(forward); // drive forward
        double detectionTime = 0;
        boolean abortion = false;
        while(true) // will move until one of the 2 conditions are met ("stage Detected"/"stage was not detected for more than 7.5 seconds - safety")
        {
            if(range <= HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES) // check for UltraSonic sensor to be over the stage platform
            {
                detectionTime = waitTimer.get();
                if(time >= detectionTime + alignmentDelay && time >= waitTime) // drive forward for waitTime and then stop
                {
                    driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false)); // stop after "waitTime"
                    break;
                }
            }
            if(time >= waitTime && safety == true && detectionTime == 0) // abort the climbing if it was trying for more than 7.5 seconds
            {
                driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false));
                // TODO: OUTPUT ABORT MESSAGE TO THE FRC DRIVE STATION CONSOLE
                abortion = true;
                break;
            }
        }
        if(abortion) // check if the stage has aborted or not
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    public boolean climbStage3(double waitTime) // retract the front pistons to lower the front wheels, over the platform
    {
        restartTimer();
        setFrontPistonsState(false);
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage4(double waitTime) // raise the back pistons
    {
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(true);
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage5(double waitTime) // be careful when using the waitTime on this
    { // move forward until the stage is detected and waitTime driven forward since detection
        restartTimer(); // restart the timer so that it is representing the time spent on this stage
        CoordinateDriveSignal forward = new CoordinateDriveSignal(speed, 0.0, 0.0, false);
        driveControl.setOpenLoopMecanum(forward); // drive forward
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage6(double waitTime) // retract the back pistons to close the climbing loop
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

    public void writeToLog()
    {

    }

    public void readPeriodicInputs()
    {

    }

    public void writePeriodicOutputs()
    {
        if(isClimbing)
        {
            if(takingTime) {
                time = waitTimer.get();
            }

            if(takingRange) {
                range = RangeSensor.getRangeInches();
            }
        }
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

    public boolean checkSystem()
    {
        return true;
    }

    public void outputTelemetry()
    {

    }

    public void stop()
    {

    }

    public void zeroSensors()
    {

    }

    public void registerEnabledLoops(ILooper enabledLooper)
    {

    }

    public static Piston_Climb getInstance()
    {
        return mInstance;
    }
}
