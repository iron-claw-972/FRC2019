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
    
    public enum currentStage {
    	STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6, ABORT;
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

    private void RestartTimer()
    {
        waitTimer.stop();
        time = 0;
        waitTimer.reset();
        waitTimer.start();
    }

    public void ClimbingManager(boolean isClimbing, int currentStage)
    {
		if (isClimbing) {
			switch (currentStage) {
			case 1:
				if (climbStage1(StageClimbTimings[0]) == true) {
					ClimbingManager(true, 2);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
					break;
			case 2:
				if (climbStage2(StageClimbTimings[1]) == true) {
					ClimbingManager(true, 3);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
				break;
			case 3:
				if (climbStage3(StageClimbTimings[2]) == true) {
					ClimbingManager(true, 4);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
				break;
			case 4:
				if (climbStage4(StageClimbTimings[3]) == true) {
					ClimbingManager(true, 5);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
				break;
			case 5:
				if (climbStage1(StageClimbTimings[4]) == true) {
					ClimbingManager(true, 6);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
				break;
			case 6:
				if (climbStage1(StageClimbTimings[0]) == true) {
					ClimbingManager(true, 2);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
				break;
			case 7:
				if (climbStage1(StageClimbTimings[0]) == true) {
					ClimbingManager(true, 2);
				} else
					ClimbingManager(false, 0);
					//This should never occur, but if it does, there's an error
				}
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
    }

    public boolean climbStage1(double waitTime)// raising the front pistons
    {
        RestartTimer(); // restart the timer so that it is representing the time spent on this stage
        setFrontPistonsState(true);
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage2(double waitTime, boolean safety) // safety is recommended while the numbers have not been calibrated as desired
    { // move forward until the stage is detected and waitTime driven forward since detection
        RestartTimer(); // restart the timer so that it is representing the time spent on this stage
        CoordinateDriveSignal forward = new CoordinateDriveSignal(1.0, 0.0, 0.0, false);
        driveControl.setOpenLoopMecanum(forward); // drive forward
        double detectionTime = 0;
        boolean abortion = false;
        while(true) // will move until one of the 2 conditions are met ("stage Detected"/"stage was not detected for more than 7.5 seconds - safety")
        {
            if(range <= HAB_LEVEL_ONE_LEVEL_TWO_DIFF_INCHES) // check for UltraSonic sensor to be over the stage platform
            {
                detectionTime = waitTimer.get();
                if(time >= detectionTime + waitTime) // drive forward for waitTime and then stop
                {
                    driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false)); // stop after "waitTime"
                    break;
                }
            }
            if(time >= waitTime + 7.5 && safety == true && detectionTime == 0) // abort the climbing if it was trying for more than 7.5 seconds
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
        RestartTimer();
        setFrontPistonsState(false);
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage4(double waitTime) // raise the back pistons
    {
        RestartTimer(); // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(true);
        while(time <= waitTime);
        return true;
    }

    public boolean climbStage5(double waitTime) // be careful when using the waitTime on this as it may move the robot over the platform
    { // move forward until the stage is detected and waitTime driven forward since detection
        RestartTimer(); // restart the timer so that it is representing the time spent on this stage
        CoordinateDriveSignal forward = new CoordinateDriveSignal(1.0, 0.0, 0.0, false);
        driveControl.setOpenLoopMecanum(forward); // drive forward
        while(time <= waitTime)
        {
            driveControl.setOpenLoopMecanum(new CoordinateDriveSignal(0,0,0, false));
        }
        return true;
    }

    public boolean climbStage6(double waitTime) // retract the back pistons to close the climbing loop
    {
        RestartTimer(); // restart the timer so that it is representing the time spent on this stage
        setBackPistonsState(false);
        while(time <= waitTime);
        return true;
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
            time = waitTimer.get();
            range = RangeSensor.getRangeInches();
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
