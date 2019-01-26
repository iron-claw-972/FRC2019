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
			Arrays.asList(ExampleSubsystem.getInstance(), Drive.getInstance()
	));

	@Override
	public void robotInit() {
	}

	@Override
	public void robotPeriodic() {
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void teleopInit() {
		Drive.getInstance().zeroSensors();

		mSubsystemManager.registerEnabledLoops(mLooper);
		mLooper.start();
	}

	@Override public void teleopPeriodic() {
		teleopManager.update();
	}

	@Override
	public void testInit() {

	}

	@Override
	public void disabledInit() {
		try {
			mLooper.stop();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

}
