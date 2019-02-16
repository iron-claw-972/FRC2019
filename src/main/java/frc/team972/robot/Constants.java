package frc.team972.robot;

import frc.team972.robot.controls.MotionProfileConstraints;

public class Constants {

    public static final double dt = 0.01;

    // CONTROLLER PORT IDS
    public static final int kDriveGamepadPort = 1;
    public static final int kElevatorJoystickPort = 999; //TODO: CHANGE THIS!!!

    // DRIVE MOTOR IDS
    // TODO: Fill with real Talon ID

    public static final double kDriveWheelRadius = 3.0;
    public static final double kDriveTicksPerRev = 4096;
    public static final double kDriveVelocityFF = 0.2; // dutyCycle per revs
    public static final double kDriveVelocityPGain = 0.1;
    public static final double kDriveVelocityDGain = 0.0;

    public static final int kLeftFrontId = 1;
    public static final int kLeftBackId = 3;
    public static final int kRightFrontId = 2;
    public static final int kRightBackId = 4;

    //ELEVATOR MOTOR IDS
    public static final int kElevatorMotorId = 999;

    //WRIST MOTOR IDS
    public static final int kWristMotorId = 999;

    //INTAKE MOTOR IDS
    public static final int kIntakeMotorId = 999;

    public static final int kLongCANTimeoutMs = 100;

    //STATE SPACE MAGICAL STUFF -- DO NOT TOUCH
    public static final int kEncoderFaultTicksAllowed = 1024;
    public static final double kEncoderFaultMinVoltage = 1.0;
    public static final double kCalibrationVoltage = -0.5; // Calibrate downwards

    public static final MotionProfileConstraints kElevatorConstraints = new MotionProfileConstraints(3.0, 3.0);
    public static final double kElevatorMaxHeight = 2.0;
    public static final double kElevatorVoltageCap = 12.0;
    public static final double kElevatorHallEffectPosition = 0.1;

    public static final MotionProfileConstraints kWristConstraints = new MotionProfileConstraints(12, 12.0);
    public static final double kWristMaxAngle = 180.0;
    public static final double kWristVoltageCap = 6.0;
    public static final double kWristHallEffectPosition = Math.toRadians(10);
    public static final int kWristEncoderCountPerRev = 1024;

    public static final MotionProfileConstraints kIntakeConstraints = new MotionProfileConstraints(12, 12.0);
    public static final double kIntakeMaxAngle = 180.0;
    public static final double kIntakeVoltageCap = 6.0;
    public static final double kIntakeHallEffectPosition = Math.toRadians(10);
    public static final int kIntakeEncoderCountPerRev = 1024;



}
