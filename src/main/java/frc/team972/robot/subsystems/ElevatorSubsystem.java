package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.HallCalibration;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.subsystems.controller.ElevatorController;

public class ElevatorSubsystem extends Subsystem {
    private static ElevatorSubsystem mInstance;
    private ElevatorController elevatorController = new ElevatorController();

    private TalonSRX mElevatorTalon;
    private VictorSPX mElevatorSlaveAVictor,mElevatorSlaveBVictor,mElevatorSlaveCVictor;
    private SensorCollection mSensorCollection;

    private HallCalibration hall_calibration_ = new HallCalibration(Constants.kElevatorHallEffectPosition);
    private boolean outputs_enabled_;

    public boolean encoder_fault_detected_;
    public double old_pos_;
    public int num_encoder_fault_ticks_ = 0;

    private double encoder_value;
    private boolean hall_status;
    private double elevator_goal_pos = 0;
    private double u = 0;

    public ElevatorSubsystem() {
        this(false);
        elevatorController.SetWeights(false);

        hall_calibration_.is_calibrated = true;
        hall_calibration_.offset = 0;
    }

    public ElevatorSubsystem(boolean test_mode) {
        if (test_mode == false) {
            mElevatorTalon = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMotorId);
            mElevatorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
            mSensorCollection = mElevatorTalon.getSensorCollection();

            mElevatorSlaveAVictor = new VictorSPX(Constants.kElevatorSlaveAMotorId);
            mElevatorSlaveBVictor = new VictorSPX(Constants.kElevatorSlaveBMotorId);
            mElevatorSlaveCVictor = new VictorSPX(Constants.kElevatorSlaveCMotorId);
        } else {
            System.out.println("ElevatorSubsystem created in Test Mode");
        }
        zeroSensors();
    }

    public void writeToLog() {

    }

    @Override
    public void fastPeriodic() {
        outputs_enabled_ = RobotState.getInstance().outputs_enabled;

        double sensor_pos_native_units = -mSensorCollection.getQuadraturePosition();
        double sensor_pos_rad = (sensor_pos_native_units / Constants.kElevatorEncoderCountPerRev) * Math.PI * 2.0 * (1.0 / 4.0);
        double sensor_pos_linear = sensor_pos_rad * Constants.kElevatorSpoolDiameter;

        //System.out.println(sensor_pos_rad + " " + elevatorController.observer_.plant_.y());

        this.setEncoder(sensor_pos_linear); // Update our sensor count so the wrist controller can read the current sensor output
        this.setHall(false); //cheat

        elevatorController.SetGoal(elevator_goal_pos);

        elevatorController.Update(this);
        u = elevatorController.getElevator_u();
        u = u * (1.0 / 12.0);

        mElevatorTalon.set(ControlMode.PercentOutput, u);
        mElevatorSlaveAVictor.set(ControlMode.PercentOutput, u);
        mElevatorSlaveBVictor.set(ControlMode.PercentOutput, u);
        mElevatorSlaveCVictor.set(ControlMode.PercentOutput, u);
    }

    @Override
    public void slowPeriodic() {
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        double sensor_pos_native_units = -mSensorCollection.getQuadraturePosition();
        double sensor_pos_rad = (sensor_pos_native_units / Constants.kElevatorEncoderCountPerRev) * Math.PI * 2.0 * (1.0 / 4.0);
        double sensor_pos_linear = sensor_pos_rad * Constants.kElevatorSpoolDiameter;

        System.out.println(sensor_pos_linear + " " + elevatorController.observer_.plant_.y().get(0,0));
        System.out.println("r=" + elevatorController.controller_.r_.get(0,0));
    }

    public void stop() {
    }

    public void zeroSensors() {
        if (mElevatorTalon != null) {
            mElevatorTalon.getSensorCollection().setQuadraturePosition(0, 100);
        }
    }

    public static ElevatorSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ElevatorSubsystem();
        }
        return mInstance;
    }

    public HallCalibration getHall_calibration_() {
        return hall_calibration_;
    }

    public boolean isOutputs_enabled_() {
        return outputs_enabled_;
    }

    public void setOutputs_enabled_(boolean outputs_enabled_) {
        this.outputs_enabled_ = outputs_enabled_;
    }

    public double getEncoder() {
        return encoder_value;
    }

    public boolean getHall() {
        return hall_status;
    }

    public void setEncoder(double encoder_value) {
        this.encoder_value = encoder_value;
    }

    public void setHall(boolean hall_status) {
        this.hall_status = hall_status;
    }

    public ElevatorController getElevatorController() {
        return elevatorController;
    }

    public double getElevator_goal_pos() {
        return elevator_goal_pos;
    }

    public void setElevator_goal_pos(double elevator_goal_pos) {
        this.elevator_goal_pos = elevator_goal_pos;
    }

    public boolean isCalibrated() {
        return hall_calibration_.is_calibrated;
    }

}