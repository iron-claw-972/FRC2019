package frc.team972.robot;

import frc.team972.robot.loops.Looper;
import frc.team972.robot.subsystems.ExampleSubsystem;
import frc.team972.robot.subsystems.SubsystemManager;
import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class RobotTest {

	@Test
	public void test() {
		assertTrue("test", true);
	}

    private Callable<Boolean> exampleVoltageMaxed() {
        return new Callable<Boolean>() {
            public Boolean call() throws Exception {
                return ExampleSubsystem.getInstance().getExampleMotorVoltage() == 6.0;
            }
        };
    }

	@Test
	public void testExampleArch() {
		ExampleSubsystem exampleSubsystem = new ExampleSubsystem(); //A new sub-system is created
		exampleSubsystem.setDesiredVoltage(1.678); //A user sets the desired voltage
		exampleSubsystem.fastPeriodic(); //Fast periodic is automatically called by the scheduling system and runs all control loops
		assertEquals(1.678 * 0.5, exampleSubsystem.getExampleMotorVoltage(), 0.001); //Assert that our logic and architecture works
	}

    @Test
    public void testExampleArchThreads() {
        SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(ExampleSubsystem.getInstance()));
        Looper mLooper = new Looper();
        mSubsystemManager.registerLoops(mLooper);
        mLooper.start();

        ExampleSubsystem.getInstance().setDesiredVoltage(12.0);

        await().atMost(1, TimeUnit.SECONDS).until(exampleVoltageMaxed());
    }

}
