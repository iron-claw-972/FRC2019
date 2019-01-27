package frc.team972.robot;

import java.io.File;
import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team972.robot.loops.Looper;
import frc.team972.robot.subsystems.Drive;
import frc.team972.robot.subsystems.ExampleSubsystem;
import frc.team972.robot.subsystems.SubsystemManager;
import frc.team972.robot.subsystems.WristSubsystem;
import frc.team972.robot.teleop.TeleopManager;
import jeigen.DenseMatrix;


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
		String userDirectory = System.getProperty("user.home");
		String nativeDirectory = userDirectory + File.separator + ".jeigen" + File.separator + "native2";
		System.out.println(nativeDirectory);
		/*
		robotState.outputs_enabled = true;
		teleopManager.update();
		mSubsystemManager.outputToSmartDashboard();
		*/

		System.out.println(System.getProperty("os.arch"));
		System.out.println(new DenseMatrix("1").mmul(new DenseMatrix("2")));

	}

	@Override
	public void testInit() {

	}

	@Override
	public void disabledInit() {
		robotState.outputs_enabled = false;
	}

}
