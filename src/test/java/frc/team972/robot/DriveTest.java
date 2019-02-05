package frc.team972.robot;

import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class DriveTest {

    @Test
    public void testMecanum() {
        DriveSignal signal = MecanumHelper.cartesianCalculate(
                new CoordinateDriveSignal(1, 0, 0, false),
                0);
        /*
        assertEquals(signal.getLeftFront(), 1, 0.0001);
        assertEquals(signal.getRightFront(), 1, 0.0001);
        assertEquals(signal.getLeftBack(), 1, 0.0001);
        assertEquals(signal.getRightBack(), 1, 0.0001);
        */
    }

}
