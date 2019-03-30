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
import frc.team972.robot.subsystems.controller.WristController;

public class WristSubsystem extends Subsystem {
    private static WristSubsystem mInstance;
    private WristController wristController = new WristController();

    private TalonSRX mWristTalon;
    private VictorSPX mWristRollerSPX;

    private SensorCollection mSensorCollection;

    private HallCalibration hall_calibration_ = new HallCalibration(Constants.kWristHallEffectPosition);
    private boolean outputs_enabled_;

    private double encoder_value;
    private boolean hall_status;
    private double wrist_goal_pos = 0;
    private double wrist_roller_goal = 0;
    private double u = 0;

    private WristSubsystem() {
        this(false);

        hall_calibration_.is_calibrated = true;
        hall_calibration_.offset = 0;
    }

    public WristSubsystem(boolean test_mode) {
        wristController.SetWeights();
        if (!test_mode) {
            mWristTalon = TalonSRXFactory.createDefaultTalon(Constants.kWristMotorId);
            mWristTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
            mSensorCollection = mWristTalon.getSensorCollection();

            mWristRollerSPX = new VictorSPX(Constants.kWristRollerMotorId);

        } else {
            System.out.println("WristSubsystem created in Test Mode");
        }
        zeroSensors();
    }

    public void writeToLog() {

    }

    @Override
    public void fastPeriodic() {
        outputs_enabled_ = RobotState.getInstance().outputs_enabled;

        double sensor_pos_native_units = mSensorCollection.getQuadraturePosition();
        double sensor_pos_rad = (sensor_pos_native_units / Constants.kWristEncoderCountPerRev) * Math.PI * 2.0 * (1.0 / 4.0);

        this.setEncoder(sensor_pos_rad); // Update our sensor count so the wrist controller can read the current sensor output
        this.setHall(false); //cheat

        wristController.SetGoal(wrist_goal_pos);

        wristController.Update(this);
        u = wristController.getWrist_u();
        u = -u * (1.0 / 12.0);

        mWristTalon.set(ControlMode.PercentOutput, u);
        mWristRollerSPX.set(ControlMode.PercentOutput, wrist_roller_goal);
    }

    @Override
    public void slowPeriodic() {
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        //System.out.println(Math.toDegrees(wristController.observer_.plant_.y().get(0,0)));

        //System.out.println(sensor_pos_linear);
        //System.out.println(sensor_pos_linear + " " + elevatorController.controller_.r_.get(0,0));
        //System.out.println("r=" + elevatorController.controller_.r_.get(0,0));
        //System.out.println(u);
    }

    public void stop() {
    }

    public void zeroSensors() {
        if (mWristTalon != null) {
            mWristTalon.getSensorCollection().setQuadraturePosition(0, 100);
        }
    }

    public static WristSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new WristSubsystem();
        }
        return mInstance;
    }

    public double getRollerCurrent() {
        return 0;
    }

    public void setRoller(double power) {
        wrist_roller_goal = power;
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

    public WristController getWristController() {
        return wristController;
    }

    public double getWrist_goal_pos() {
        return wrist_goal_pos;
    }

    public void setWrist_goal_pos(double wrist_goal_pos) {
        this.wrist_goal_pos = wrist_goal_pos;
    }

    public boolean isCalibrated() {
        return hall_calibration_.is_calibrated;
    }

}