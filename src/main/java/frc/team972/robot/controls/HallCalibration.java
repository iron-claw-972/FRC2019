package frc.team972.robot.controls;

public class HallCalibration {
    public boolean is_calibrated = false;
    public double offset = 0;
    private double max_hall_true_ = 0;
    private double min_hall_true_ = 0;
    private double max_overall_ = 0;
    private double min_overall_ = 0;
    private boolean first_time_ = true;
    private boolean magnet_found_ = false;
    private double magnet_position;

    public HallCalibration(double magnet_position) {
        this.magnet_position = magnet_position;
    }

    public boolean is_calibrated() {
        return is_calibrated;
    }

    public double getCorrectedValue(double main_sensor_value) {
        return main_sensor_value + offset;
    }

    public double Update(double main_sensor_value, boolean hall_value) {
        if (hall_value) {
            // Update the max and min values for when the hall sensor is triggered. Set
            // them to the current value if it is the first time seeing the magnet.

            if (main_sensor_value > max_hall_true_ || !magnet_found_) {
                max_hall_true_ = main_sensor_value;
            }
            if (main_sensor_value < min_hall_true_ || !magnet_found_) {
                min_hall_true_ = main_sensor_value;
            }
            magnet_found_ = true;
        }
        // Update the max and min overall values. Set the to the current value if
        // it is the first time running Update()
        if (main_sensor_value > max_overall_ || first_time_) {
            max_overall_ = main_sensor_value;
        }
        if (main_sensor_value < min_overall_ || first_time_) {
            min_overall_ = main_sensor_value;
        }
        // Return the best estimate known. If the magnet is not found or the edges of
        // the magnet's range have not been reached, there is no best estimate. In
        // the event that calibrated_ is true, do not set it to false even if the
        // condition is not currently met, as that could reset any portions of code
        // that assume calibration is complete.

        if ((magnet_found_ && max_overall_ > max_hall_true_ &&
                min_overall_ < min_hall_true_) ||
                is_calibrated) {
            // The center of the magnet's range is magnet_position_, so the offset if
            // magnet_position_ - the raw value of the center of the magnet
            offset = magnet_position - (max_hall_true_ + min_hall_true_) / 2;
            is_calibrated = true;
        }

        first_time_ = false;
        return main_sensor_value + offset;
    }
}