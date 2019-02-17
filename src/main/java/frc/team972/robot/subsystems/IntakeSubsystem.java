package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.HallCalibration;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.subsystems.controller.IntakeController;

public class IntakeSubsystem extends Subsystem {

    private static IntakeSubsystem mInstance;
    private IntakeController IntakeController = new IntakeController();

    private TalonSRX mIntakeTalon;
    private TalonSRX mRollerTalon;

    private SensorCollection mSensorCollection;

    private HallCalibration hall_calibration_ = new HallCalibration(Constants.kIntakeHallEffectPosition);
    private boolean outputs_enabled_;

    private double encoder_value;
    private boolean hall_status;
    private double Intake_goal_pos = 0;
    private double u = 0;
    private double roller_power = 0;

    private IntakeSubsystem() {
        this(false);

        //TOOD: cheat calibration for testing
        hall_calibration_.is_calibrated = true;
        hall_calibration_.offset = 0;
    }

    public IntakeSubsystem(boolean test_mode) {
        IntakeController.SetWeights();
        if (!test_mode) {
            mIntakeTalon = TalonSRXFactory.createDefaultTalon(Constants.kIntakeMotorId);
            mIntakeTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);

            mRollerTalon = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRollerMotorId);

            mSensorCollection = mIntakeTalon.getSensorCollection();
        } else {
            System.out.println("IntakeSubsystem created in Test Mode");
        }
        zeroSensors();
    }

    public void writeToLog() {

    }

    @Override
    public void fastPeriodic() {
        outputs_enabled_ = RobotState.getInstance().outputs_enabled;

        double sensor_pos_native_units = mSensorCollection.getQuadraturePosition();
        double sensor_pos_rad = (sensor_pos_native_units / Constants.kIntakeEncoderCountPerRev) * Math.PI * 2.0 * (1.0 / 4.0);

        this.setEncoder(sensor_pos_rad); // Update our sensor count so the Intake controller can read the current sensor output
        this.setHall(false); //cheat

        IntakeController.SetGoal(Intake_goal_pos);

        IntakeController.Update(this);
        u = IntakeController.getIntake_u();
        u = u * (1.0 / 12.0);
        u = u * -1.0;

        mIntakeTalon.set(ControlMode.PercentOutput, u);
        mRollerTalon.set(ControlMode.PercentOutput, roller_power);
    }

    @Override
    public void slowPeriodic() {
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        //System.out.println(u + " " + Intake_goal_pos + " " + this.getEncoder() + " " + IntakeController.observer_.plant_.y().get(0, 0));
        System.out.println(roller_power);
    }

    public void stop() {
    }

    public void zeroSensors() {
        if (mIntakeTalon != null) {
            mIntakeTalon.getSensorCollection().setQuadraturePosition(0, 100);
        }
    }

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
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

    public IntakeController getIntakeController() {
        return IntakeController;
    }

    public double getIntake_goal_pos() {
        return Intake_goal_pos;
    }

    public void setIntake_goal_pos(double Intake_goal_pos) {
        this.Intake_goal_pos = Intake_goal_pos;
    }

    public boolean isCalibrated() {
        return hall_calibration_.is_calibrated;
    }

    public void setRollerPower(double power) {
        roller_power = power;
    }

}