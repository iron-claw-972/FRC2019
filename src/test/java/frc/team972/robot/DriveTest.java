package frc.team972.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team972.robot.subsystems.Drive;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;

import static org.junit.Assert.assertEquals;

public class DriveTest {

    //TODO: write proper unit test
    @Test
    public void testMecanum() {
//        DriveSignal signal = MecanumHelper.cartesianCalculate(
//                new CoordinateDriveSignal(1, 5, 20),
//                30);
//        //values generated on Lua test-bench
//        assertEquals(signal.getLeftFront(), 23.196152422707, 0.0001);
//        assertEquals(signal.getRightFront(), -13.535898384862, 0.0001);
//        assertEquals(signal.getLeftBack(), 26.464101615138, 0.0001);
//        assertEquals(signal.getRightBack(), -16.803847577293, 0.0001);
    }

}
