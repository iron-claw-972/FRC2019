package frc.team972.robot;

import frc.team972.robot.lib.*;
import frc.team972.robot.subsystems.DriveSubsystem;

import java.util.Map;

public class RobotState {
    public boolean outputs_enabled = false;
    private static RobotState instance = new RobotState();

    public static RobotState getInstance() {
        return instance;
    }

    private static final int kObservationBufferSize = 100;

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;

    private RobotState() {
        reset(0, new Pose2d());
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        DriveSubsystem.getInstance().zeroSensors();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }
}