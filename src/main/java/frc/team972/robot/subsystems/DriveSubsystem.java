package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;

public class DriveSubsystem extends Subsystem {

    static private AHRS ahrs;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private RobotState robotState = RobotState.getInstance();
    private DriveControlState mDriveControlState;
    private TalonSRX mLeftFront, mLeftBack, mRightFront, mRightBack;
    private Double last_angle = null;

    CoordinateDriveSignal mecanumDriveSignalDesired = null;
    Pose2d mecanumDrivePoseDesired = null;

    private boolean mIsBrakeMode;
    private static DriveSubsystem mInstance = null;

    private double left_error_old = 0;
    private double right_error_old = 0;
    private double left_b_error_old = 0;
    private double right_b_error_old = 0;

    public DriveSubsystem() {
        mLeftFront = TalonSRXFactory.createDefaultTalon(Constants.kLeftFrontId);
        configureMaster(mLeftFront);

        mLeftBack = TalonSRXFactory.createDefaultTalon(Constants.kLeftBackId);
        configureMaster(mLeftBack);

        mRightFront = TalonSRXFactory.createDefaultTalon(Constants.kRightFrontId);
        configureMaster(mRightFront);

        mRightBack = TalonSRXFactory.createDefaultTalon(Constants.kRightBackId);
        configureMaster(mRightBack);

        mIsBrakeMode = true;
        setBrakeMode(true);
    }

    public static DriveSubsystem getInstance() {
        if (mInstance == null) {
            ahrs = new AHRS(SPI.Port.kMXP, (byte) 200);
            mInstance = new DriveSubsystem();
        }
        return mInstance;
    }


    private void configureMaster(TalonSRX talon) {
        //TODO: Configure Talons for Sensored operation
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        /*
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect encoder: " + sensorPresent, false);
        }
        */

        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.01, 0);
    }

    public synchronized void setMecanumDrivePoseDesired(Pose2d pose) {
        mecanumDrivePoseDesired = pose;
    }

    public synchronized void setOpenLoopMecanum(CoordinateDriveSignal signal) {
        mecanumDriveSignalDesired = signal;
        if (mDriveControlState != DriveControlState.OPEN_LOOP_MECANUM) {
            setBrakeMode(true);

            mDriveControlState = DriveControlState.OPEN_LOOP_MECANUM;
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);

            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_front_demand = signal.getLeftFront();
        mPeriodicIO.right_front_demand = signal.getRightFront();
        mPeriodicIO.left_back_demand = signal.getLeftBack();
        mPeriodicIO.right_back_demand = signal.getRightBack();
    }

    public synchronized void setCloseLoopMecanum(CoordinateDriveSignal signal) {
        mecanumDriveSignalDesired = signal;
        if (mDriveControlState != DriveControlState.CLOSED_LOOP_MECANUM) {
            setBrakeMode(true);

            mDriveControlState = DriveControlState.CLOSED_LOOP_MECANUM;
        }
    }

    public synchronized void setCloseLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.CLOSED_LOOP) {
            setBrakeMode(true);

            mDriveControlState = DriveControlState.CLOSED_LOOP;
        }

        DriveSensorReading sensorReading = encoderToRealUnits(readEncodersVelocity());

        double left_error = (signal.getLeftFront() - sensorReading.left);
        double right_error = (signal.getRightFront() + sensorReading.right);
        double left_b_error = (signal.getLeftBack() - sensorReading.left_back);
        double right_b_error = (signal.getRightBack() + sensorReading.right_back);

        mPeriodicIO.left_front_demand = (signal.getLeftFront() * Constants.kDriveVelocityFF) + left_error * Constants.kDriveVelocityPGain + (left_error - left_error_old) * Constants.kDriveVelocityDGain;
        mPeriodicIO.right_front_demand = (signal.getRightFront() * Constants.kDriveVelocityFF) + right_error * Constants.kDriveVelocityPGain + (right_error - right_error_old) * Constants.kDriveVelocityDGain;
        mPeriodicIO.right_back_demand = (signal.getRightBack() * Constants.kDriveVelocityFF) + right_b_error * Constants.kDriveVelocityPGain + (left_b_error - left_b_error_old) * Constants.kDriveVelocityDGain;
        mPeriodicIO.left_back_demand = (signal.getLeftBack() * Constants.kDriveVelocityFF) + left_b_error * Constants.kDriveVelocityPGain + (right_b_error - right_b_error_old) * Constants.kDriveVelocityDGain;

        left_error_old = left_error;
        right_error_old = right_error;
        left_b_error_old = left_b_error;
        right_b_error_old = right_b_error;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

            mLeftBack.setNeutralMode(mode);
            mLeftFront.setNeutralMode(mode);
            mRightBack.setNeutralMode(mode);
            mRightFront.setNeutralMode(mode);
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public enum DriveControlState {
        OPEN_LOOP, // voltage control
        PATH_FOLLOWING,
        OPEN_LOOP_MECANUM,
        CLOSED_LOOP,
        CLOSED_LOOP_MECANUM
    }

    @Override
    public boolean checkSystem() {
        //TODO: Implement
        return true;
    }

    @Override
    public synchronized void fastPeriodic() {
        if (RobotState.getInstance().outputs_enabled == false) {
            setOpenLoop(new DriveSignal(0, 0, 0, 0));
            return;
        }

        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setMotorsOpenValue();
        } else if ((mDriveControlState == DriveControlState.PATH_FOLLOWING) && (mecanumDrivePoseDesired != null)) {
            //path following mode
            Pose2d current_state = robotState.getLatestFieldToVehicle().getValue();

            System.out.println(current_state.getTranslation().x() - mecanumDrivePoseDesired.getTranslation().x());

            double x_p = 0;
            double y_p = 0;
            double rotate_p = 0;

            this.setCloseLoopMecanum(
                    MecanumHelper.mecanumDrive(x_p, y_p, rotate_p, false)
            );

        } else if ((mDriveControlState == DriveControlState.OPEN_LOOP_MECANUM) && (mecanumDriveSignalDesired != null)) {
            double current_angle = -ahrs.getAngle(); //TODO: Grab estimated robot rotation state from RobotState after sensor-fusion.

            if (mecanumDriveSignalDesired.getFieldOrient() == false) {
                current_angle = 0;
            }

            if (last_angle == null) {
                //Zero Last angle on first loop
                last_angle = current_angle;
            }
            double angle_velocity = (current_angle - last_angle);
            double angle_correction = -angle_velocity * 0.2;
            angle_correction = MecanumHelper.handleDeadband(angle_correction, 0.01);

            if (current_angle != 0) {
                mecanumDriveSignalDesired.addRotation(angle_correction);
            }

            DriveSignal driveSignal = MecanumHelper.cartesianCalculate(mecanumDriveSignalDesired, current_angle);

            //Feed transformed Mecanum values into traditional motor values

            setOpenLoop(driveSignal);
            setMotorsOpenValue();

            if (mecanumDriveSignalDesired.getFieldOrient()) {
                last_angle = current_angle;
            } else {
                last_angle = null;
            }
        } else if ((mDriveControlState == DriveControlState.CLOSED_LOOP_MECANUM) && (mecanumDriveSignalDesired != null)) {
            double current_angle = -ahrs.getAngle();
            DriveSignal driveSignal = MecanumHelper.cartesianCalculate(mecanumDriveSignalDesired, current_angle);

            //Feed transformed Mecanum values into traditional motor values

            setCloseLoop(driveSignal); // Calculate u's
            setMotorsOpenValue();
        }
    }

    public void setMotorsOpenValue() {
        mRightFront.set(ControlMode.PercentOutput, mPeriodicIO.right_front_demand, DemandType.ArbitraryFeedForward, 0.0);
        mLeftFront.set(ControlMode.PercentOutput, mPeriodicIO.left_front_demand, DemandType.ArbitraryFeedForward, 0.0);
        mRightBack.set(ControlMode.PercentOutput, mPeriodicIO.right_back_demand, DemandType.ArbitraryFeedForward, 0.0);
        mLeftBack.set(ControlMode.PercentOutput, mPeriodicIO.left_back_demand, DemandType.ArbitraryFeedForward, 0.0);
    }

    @Override
    public void outputTelemetry() {
        //System.out.println(encoderToRealUnits(readEncodersPosition()));
    }

    @Override
    public void zeroSensors() {
        ahrs.reset();
        mLeftBack.getSensorCollection().setQuadraturePosition(0, Constants.kLongCANTimeoutMs);
        mRightBack.getSensorCollection().setQuadraturePosition(0, Constants.kLongCANTimeoutMs);
        mLeftFront.getSensorCollection().setQuadraturePosition(0, Constants.kLongCANTimeoutMs);
        mRightFront.getSensorCollection().setQuadraturePosition(0, Constants.kLongCANTimeoutMs);
        last_angle = null;
    }

    public DriveSensorReading readEncodersPosition() {
        DriveSensorReading sensorReading = new DriveSensorReading();
        sensorReading.left = -mLeftFront.getSensorCollection().getQuadraturePosition();
        sensorReading.right = mRightFront.getSensorCollection().getQuadraturePosition();
        sensorReading.left_back = -mLeftBack.getSensorCollection().getQuadraturePosition();
        sensorReading.right_back = mRightBack.getSensorCollection().getQuadraturePosition();

        return sensorReading;
    }

    public DriveSensorReading readEncodersVelocity() {
        DriveSensorReading sensorReading = new DriveSensorReading();

        sensorReading.left = -mLeftFront.getSensorCollection().getQuadratureVelocity() * 10;
        sensorReading.right = mRightFront.getSensorCollection().getQuadratureVelocity() * 10;
        sensorReading.left_back = -mLeftBack.getSensorCollection().getQuadratureVelocity() * 10;
        sensorReading.right_back = mRightBack.getSensorCollection().getQuadratureVelocity() * 10;

        return sensorReading;
    }

    public DriveSensorReading encoderToRealUnits(DriveSensorReading sensorReading) {
        sensorReading.left = sensorReading.left * (1 / Constants.kDriveTicksPerRev);
        sensorReading.right = sensorReading.right * (1 / Constants.kDriveTicksPerRev);
        sensorReading.left_back = sensorReading.left_back * (1 / Constants.kDriveTicksPerRev);
        sensorReading.right_back = sensorReading.right_back * (1 / Constants.kDriveTicksPerRev);

        return sensorReading;
    }

    public DriveSensorReading realUnitsToLinear(DriveSensorReading sensorReading) {
        sensorReading.left = sensorReading.left * 2 * Math.PI * Constants.kDriveWheelRadius;
        sensorReading.right = sensorReading.right * 2 * Math.PI * Constants.kDriveWheelRadius;
        sensorReading.left_back = sensorReading.left_back * 2 * Math.PI * Constants.kDriveWheelRadius;
        sensorReading.right_back = sensorReading.right_back * 2 * Math.PI * Constants.kDriveWheelRadius;

        return sensorReading;
    }

    public double getHeading() {
        return ahrs.getAngle();
    }

    @Override
    public void stop() {
        mecanumDriveSignalDesired = null;
        setOpenLoop(new DriveSignal(0, 0, 0, 0));
    }

    public static class PeriodicIO {
        // OUTPUTS
        public double left_front_demand;
        public double right_front_demand;
        public double left_back_demand;
        public double right_back_demand;

        public String toString() {
            return left_back_demand + " " + right_front_demand + " " + left_back_demand + " " + right_back_demand;
        }
    }

    public static class DriveSensorReading {
        public double left = 0;
        public double left_back = 0;
        public double right = 0;
        public double right_back = 0;

        public String toString() {
            return left + " " + right + " " + left_back + " " + right_back;
        }
    }

}


