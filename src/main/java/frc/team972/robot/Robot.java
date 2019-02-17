package frc.team972.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.team972.robot.loops.Looper;
import frc.team972.robot.subsystems.*;
import frc.team972.robot.teleop.TeleopManager;

import java.util.Arrays;

public class Robot extends TimedRobot {

	private TeleopManager teleopManager = TeleopManager.getInstance();
	private Looper mLooper = new Looper();

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(DriveSubsystem.getInstance(), RobotStateEstimator.getInstance(), IntakeSubsystem.getInstance()
	));

	private RobotState robotState = RobotState.getInstance();

	@Override
	public void robotInit() {
		mSubsystemManager.registerLoops(mLooper); // This loop runs FOREVER!
		mLooper.start();
		Shuffleboard.startRecording();
	}

	@Override
	public void robotPeriodic() {
		if(this.isDisabled()) {
			robotState.outputs_enabled = false;
		}
	}

	@Override
	public void autonomousInit() {
		robotState.outputs_enabled = true;
	}

	@Override
	public void teleopInit() {
		robotState.outputs_enabled = true;
		DriveSubsystem.getInstance().zeroSensors();
		RobotStateEstimator.getInstance().reset();
		IntakeSubsystem.getInstance().zeroSensors();
	}

	@Override
	public void teleopPeriodic() {
		teleopManager.update();
		mSubsystemManager.slowPeriodic();
		mSubsystemManager.outputToSmartDashboard();
	}

	@Override
	public void testInit() {

	}

	@Override
	public void disabledInit() {
		robotState.outputs_enabled = false;
	}

}
