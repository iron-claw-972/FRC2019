package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.team972.robot.Constants;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.teleop.ControlBoard;
import frc.team972.robot.util.CoordinateDriveSignal;
import frc.team972.robot.util.DriveSignal;
import frc.team972.robot.util.MecanumHelper;

public class Drive extends Subsystem {

    static private AHRS ahrs;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DriveControlState mDriveControlState;
    private TalonSRX mLeftFront, mLeftBack, mRightFront, mRightBack;
    private Double last_angle = null;

    CoordinateDriveSignal mecanumDriveSignalDesired = null;

    private boolean mIsBrakeMode;
    private static Drive mInstance = null;

    public Drive() {
        /*
        kLeftFront = new VictorSPX(Constants.kLeftFrontId);
        kLeftBack = new VictorSPX(Constants.kLeftBackId);
        kRightFront = new VictorSPX(Constants.kRightFrontId);
        kRightBack = new VictorSPX(Constants.kRightBackId);
        */


        mLeftFront = TalonSRXFactory.createDefaultTalon(Constants.kLeftFrontId);
        configureMaster(mLeftFront, true);

        mLeftBack = TalonSRXFactory.createDefaultTalon(Constants.kLeftBackId);
        configureMaster(mLeftBack, true);

        mRightFront = TalonSRXFactory.createDefaultTalon(Constants.kRightFrontId);
        configureMaster(mRightFront, false);

        mRightBack = TalonSRXFactory.createDefaultTalon(Constants.kRightBackId);
        configureMaster(mRightBack, false);

        mIsBrakeMode = true;
        setBrakeMode(true);
    }

    public static Drive getInstance() {
        if (mInstance == null) {
            ahrs = new AHRS(SPI.Port.kMXP, (byte)200);
            mInstance = new Drive();
        }
        return mInstance;
    }


    private void configureMaster(TalonSRX talon, boolean left) {
        //TODO: Configure Talons for Sensored operation
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        /*final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        if (sensorPresent != ErrorCode.OK) {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        */
        //talon.setInverted(!left);
        //talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        //talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        //talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        //talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    public synchronized  void setOpenLoopMecanum(CoordinateDriveSignal signal) {
        mecanumDriveSignalDesired = signal;
        if(mDriveControlState != DriveControlState.OPEN_LOOP_MECANUM) {
            setBrakeMode(true);

            //System.out.println("Switching to mecanum open loop");
            mDriveControlState = DriveControlState.OPEN_LOOP_MECANUM;
        }
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);

            //System.out.println("Switching to open loop");
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_front_demand = signal.getLeftFront();
        mPeriodicIO.right_front_demand = signal.getRightFront();
        mPeriodicIO.left_back_demand = signal.getLeftBack();
        mPeriodicIO.right_back_demand = signal.getRightBack();
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
        PATH_FOLLOWING, // velocity control
        OPEN_LOOP_MECANUM
    }

    @Override
    public boolean checkSystem() {
        //TODO: Implement
        return true;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            setMotorsOpenValue();
        } else if((mDriveControlState == DriveControlState.OPEN_LOOP_MECANUM) && (mecanumDriveSignalDesired != null)) {
            double current_angle = -ahrs.getAngle(); //TODO: Grab estimated robot rotation state from RobotState after sensor-fusion.

            if(mecanumDriveSignalDesired.getFieldOrient() == false) {
                current_angle = 0;
            }

            if(last_angle == null) {
                //Zero Last angle on first loop
                last_angle = current_angle;
            }
            double angle_velocity = (current_angle - last_angle);
            double angle_correction = -angle_velocity * 0.2;
            angle_correction = MecanumHelper.handleDeadband(angle_correction, 0.01);

            System.out.println(current_angle + " " + last_angle + " " + mecanumDriveSignalDesired.getFieldOrient());

            if(current_angle != 0) {
                mecanumDriveSignalDesired.addRotation(angle_correction);
            }

            DriveSignal driveSignal = MecanumHelper.cartesianCalculate(mecanumDriveSignalDesired, current_angle);

            //Feed transformed Mecanum values into traditional motor values

            setOpenLoop(driveSignal);
            setMotorsOpenValue();

            if(mecanumDriveSignalDesired.getFieldOrient()) {
                last_angle = current_angle;
            } else {
                last_angle = null;
            }
        }
    }

    public void setMotorsOpenValue() {

        //System.out.println(mPeriodicIO);

        mRightFront.set(ControlMode.PercentOutput, mPeriodicIO.right_front_demand, DemandType.ArbitraryFeedForward, 0.0);
        mLeftFront.set(ControlMode.PercentOutput, mPeriodicIO.left_front_demand, DemandType.ArbitraryFeedForward, 0.0);
        mRightBack.set(ControlMode.PercentOutput, mPeriodicIO.right_back_demand, DemandType.ArbitraryFeedForward, 0.0);
        mLeftBack.set(ControlMode.PercentOutput, mPeriodicIO.left_back_demand, DemandType.ArbitraryFeedForward, 0.0);
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void zeroSensors() {
        ahrs.reset();
        last_angle = null;
    }

    @Override
    public void stop() {
        mecanumDriveSignalDesired = null;
        setOpenLoop(new DriveSignal(0,0,0,0));
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

}


