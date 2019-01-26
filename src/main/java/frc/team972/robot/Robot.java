package frc.team972.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team972.robot.loops.Looper;
import frc.team972.robot.subsystems.Drive;
import frc.team972.robot.subsystems.ExampleSubsystem;
import frc.team972.robot.subsystems.SubsystemManager;
import frc.team972.robot.teleop.TeleopManager;

public class Robot extends TimedRobot {

	private TeleopManager teleopManager = TeleopManager.getInstance();
	private Looper mLooper = new Looper();

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(Drive.getInstance()
	));

	private RobotState robotState;

	@Override
	public void robotInit() {
		mSubsystemManager.registerEnabledLoops(mLooper);
		mLooper.start();
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
		robotState.outputs_enabled = true;
	}

	@Override
	public void teleopInit() {
		robotState.outputs_enabled = true;
	}

	@Override public void teleopPeriodic() {
		robotState.outputs_enabled = true;
		teleopManager.update();
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
