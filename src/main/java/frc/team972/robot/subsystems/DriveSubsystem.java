package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.team972.robot.Constants;
import frc.team972.robot.RobotState;
import frc.team972.robot.controls.MotionProfilePosition;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.lib.Pose2d;
import frc.team972.robot.lib.Util;
import frc.team972.robot.statemachines.DriveStateMachine;
import frc.team972.robot.subsystems.controller.DriveMotorController;
import frc.team972.robot.subsystems.controller.DriveMotorVelocitySimpleController;
import frc.team972.robot.subsystems.controller.MecanumAngleLockController;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;
import jeigen.DenseMatrix;

public class DriveSubsystem extends Subsystem {

    static private AHRS ahrs;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DriveControlState mDriveControlState;
    private TalonSRX mLeftFront, mLeftBack, mRightFront, mRightBack;
    private Double last_angle = null;

    CoordinateDriveSignal mecanumDriveSignalDesired = null;
    private DriveStateMachine driveStateMachine = new DriveStateMachine(this);

    private boolean mIsBrakeMode;
    private static DriveSubsystem mInstance = null;

    /*
    private DriveMotorVelocitySimpleController left_c = new DriveMotorVelocitySimpleController(Constants.kDriveVelocityPGain, Constants.kDriveVelocityFF, 0);
    private DriveMotorVelocitySimpleController left_b_c = new DriveMotorVelocitySimpleController(Constants.kDriveVelocityPGain, Constants.kDriveVelocityFF, 0);
    private DriveMotorVelocitySimpleController right_c = new DriveMotorVelocitySimpleController(Constants.kDriveVelocityPGain, Constants.kDriveVelocityFF, 0);
    private DriveMotorVelocitySimpleController right_b_c = new DriveMotorVelocitySimpleController(Constants.kDriveVelocityPGain, Constants.kDriveVelocityFF, 0);
    */

    private DriveMotorController left_c = new DriveMotorController();
    private DriveMotorController left_b_c = new DriveMotorController();
    private DriveMotorController right_c = new DriveMotorController();
    private DriveMotorController right_b_c = new DriveMotorController();

    private MecanumAngleLockController angleLockController = new MecanumAngleLockController();

    public DriveSubsystem(boolean test_mode) {
        if (!test_mode) {
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
            zeroSensors();
        } else {
            System.out.println("DriveSubsystem created in Test Mode");
        }
    }

    public DriveSubsystem() {
        this(false);
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
        if (pose == null) {
            driveStateMachine.requestManual();
        } else {
            driveStateMachine.requestNewPath(pose);
            if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
                setBrakeMode(true);
                mDriveControlState = DriveControlState.PATH_FOLLOWING;
            }
        }
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

        double driveMagnitude = Constants.kDriveVelocityCap;

        DriveSensorReading sensorReadingPosition = encoderToRealUnits(readEncodersPosition());
        DriveSensorReading sensorReadingVelocity = encoderToRealUnits(readEncodersVelocity());

        //set motion goals to current position and desired velocity
        //(we might have to gain schedule K matrix to [0, velocity gain] incase this trick doesn't work)
        left_c.SetGoal(new MotionProfilePosition(sensorReadingPosition.left,signal.getLeftFront() * driveMagnitude));
        left_b_c.SetGoal(new MotionProfilePosition(sensorReadingPosition.left_back,signal.getLeftBack() * driveMagnitude));
        right_c.SetGoal(new MotionProfilePosition(sensorReadingPosition.right,signal.getRightFront() * driveMagnitude));
        right_b_c.SetGoal(new MotionProfilePosition(sensorReadingPosition.right_back,signal.getRightBack() * driveMagnitude));

        double l_p = left_c.Update(sensorReadingPosition.left);
        double l_b_p = left_b_c.Update(sensorReadingPosition.left_back);
        double r_p = right_c.Update(sensorReadingPosition.right);
        double r_b_p = right_b_c.Update(sensorReadingPosition.right_back);

        mPeriodicIO.left_front_demand = l_p;
        mPeriodicIO.left_back_demand = l_b_p;
        mPeriodicIO.right_front_demand = r_p;
        mPeriodicIO.right_back_demand = r_b_p;
    }

    public synchronized void setBrakeMode(boolean on) {
        if (mIsBrakeMode != on) {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;

            if (mLeftBack != null) {
                mLeftBack.setNeutralMode(mode);
                mLeftFront.setNeutralMode(mode);
                mRightBack.setNeutralMode(mode);
                mRightFront.setNeutralMode(mode);
            }
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public PeriodicIO getPeriodicIO() {
        return mPeriodicIO;
    }

    public DriveControlState getDriveControlState() {
        return mDriveControlState;
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
    public synchronized void fastPeriodic(double timestamp) {
        if (RobotState.getInstance().outputs_enabled == false) {
            setOpenLoop(new DriveSignal(0, 0, 0, 0));
            return;
        }

        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setMotorsOpenValue();
        } else if ((mDriveControlState == DriveControlState.PATH_FOLLOWING)) {
            //path following mode

            Pose2d current_state = RobotState.getInstance().getLatestFieldToVehicle().getValue();
            DenseMatrix commanded_full_state = driveStateMachine.update(timestamp); // full state matrix that the robot should be at

            //TODO: implement control law to turn desired matrix [position, velocity] state into a desired [velocity] state to converge our goal optimally
            double x_p = 0;
            double y_p = 0;
            double rotate_p = 0;

            if(commanded_full_state != null) {
                x_p = -(commanded_full_state.get(0,0) - current_state.getTranslation().x()) * 0.1;
                y_p = (commanded_full_state.get(0,1) - current_state.getTranslation().y()) * 0.1;

                x_p = Util.limit(x_p, 0.25);
                y_p = Util.limit(y_p, 0.25);

                System.out.println(x_p + " " + y_p);
                System.out.println(current_state.getTranslation());
            }

            this.setCloseLoopMecanum(
                    MecanumHelper.mecanumDrive(x_p, y_p, rotate_p, false)
            );
            setMotorsOpenValue();
        } else if ((mDriveControlState == DriveControlState.OPEN_LOOP_MECANUM) && (mecanumDriveSignalDesired != null)) {
            double current_angle = 0;

            if (mecanumDriveSignalDesired.getFieldOrient()) {
                current_angle = -ahrs.getAngle();
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
            double current_angle = 0;
            if (mecanumDriveSignalDesired.getFieldOrient()) {
                current_angle = -ahrs.getAngle();
            }

            double rotation_power = angleLockController.update(current_angle, mecanumDriveSignalDesired.getRotation() * 25);
            mecanumDriveSignalDesired.setRotation(rotation_power);

            DriveSignal driveSignal = MecanumHelper.cartesianCalculate(mecanumDriveSignalDesired, current_angle);

            setCloseLoop(driveSignal);
            setMotorsOpenValue();
        }
    }

    public void setMotorsOpenValue() {
        if (mRightFront != null) {
            mRightFront.set(ControlMode.PercentOutput, mPeriodicIO.right_front_demand, DemandType.ArbitraryFeedForward, 0.0);
            mLeftFront.set(ControlMode.PercentOutput, mPeriodicIO.left_front_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightBack.set(ControlMode.PercentOutput, mPeriodicIO.right_back_demand, DemandType.ArbitraryFeedForward, 0.0);
            mLeftBack.set(ControlMode.PercentOutput, mPeriodicIO.left_back_demand, DemandType.ArbitraryFeedForward, 0.0);
        }
    }

    @Override
    public void outputTelemetry() {
        //System.out.println(encoderToRealUnits(readEncodersPosition()));
    }

    @Override
    public void zeroSensors() {
        ahrs.reset();
        angleLockController.reset();
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


