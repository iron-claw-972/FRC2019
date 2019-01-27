package frc.team972.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team972.robot.loops.Looper;
import frc.team972.robot.subsystems.SubsystemManager;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.teleop.TeleopManager;

import java.util.Arrays;

public class Robot extends TimedRobot {

	private TeleopManager teleopManager = TeleopManager.getInstance();
	private Looper mLooper = new Looper();

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(WristSubsystem.getInstance()
	));

	private RobotState robotState = RobotState.getInstance();

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
