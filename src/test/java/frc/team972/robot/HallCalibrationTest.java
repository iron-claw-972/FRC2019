package frc.team972.robot;

import frc.team972.robot.controls.HallCalibration;
import org.junit.Assert;
import org.junit.Test;

public class HallCalibrationTest {

    public HallCalibration calibration_ = new HallCalibration(0);

    // Update the calibration assuming sensors are at the same position
    void UpdateTest(double position) {
        // The magnet's range is 100 to 200 exclusive
        if (position > 100 && position < 200) {
            calibration_.Update(position, true);
        } else {
            calibration_.Update(position, false);
        }
    }
    // Update the calibration allowing for sensors being at different positions
    void UpdateTest(double main_sensor_position, double hall_sensor_position) {
        if (hall_sensor_position > 100 && hall_sensor_position < 200) {
            calibration_.Update(main_sensor_position, true);
        } else {
            calibration_.Update(main_sensor_position, false);
        }
    }

    boolean is_calibrated() { return calibration_.is_calibrated(); }

    // Get the offsetted value from a raw sensor value
    double ValueAt(double position) {
        return calibration_.Update(position, false);
    }

    @Test
    public void testInit() {
        calibration_ = new HallCalibration(0);
        Assert.assertFalse(is_calibrated());
    }

    @Test
    public void testCalibrateUp() {
        calibration_ = new HallCalibration(0.0);
        for (int i = 0; i < 200; i++) {
            UpdateTest(i);
            Assert.assertFalse(is_calibrated());
        }
        UpdateTest(200);
        Assert.assertTrue(is_calibrated());
    }

    @Test
    public void testCalibrateDown() {
        calibration_ = new HallCalibration(0.0);
        for (int i = 300; i > 100; i--) {
            UpdateTest(i);
            Assert.assertFalse(is_calibrated());
        }
        UpdateTest(100);
        Assert.assertTrue(is_calibrated());
    }
}
