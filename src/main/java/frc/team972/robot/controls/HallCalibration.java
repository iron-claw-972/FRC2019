package frc.team972.robot.controls;

public class HallCalibration {
    public boolean is_calibrated = false;
    public double offset = 0;

    public boolean is_calibrated() {
        return is_calibrated;
    }

    //TODO: Implement
    public double Update(double main_sensor_value, boolean hall_value) {
        return main_sensor_value;
    }
}