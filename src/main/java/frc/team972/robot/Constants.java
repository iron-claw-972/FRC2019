package frc.team972.robot;

import frc.team972.robot.controls.MotionProfileConstraints;

public class Constants {

    // CONTROLLER PORT IDS
    public static final int kDriveGamepadPort = 1;
    public static final int kElevatorJoystickPort = 999; //TODO: CHANGE THIS!!!

    // DRIVE MOTOR IDS
    // TODO: Fill with real Talon ID

    public static final int kLeftFrontId = 1;
    public static final int kLeftBackId = 3;
    public static final int kRightFrontId = 2;
    public static final int kRightBackId = 4;

    //ELEVATOR MOTOR IDS
    public static final int kElevatorMotorId = 999;
  
    //ARM MOTOR IDS
    public static final int mArmTalonId = -1;
    public static final int kWristMotorId = 2;

    public static final int kLongCANTimeoutMs = 100;
    public static final double kDriveVoltageRampRate = 0.0;

    //STATE SPACE MAGICAL STUFF
    public static final int kEncoderFaultTicksAllowed = 1024;
    public static final double kEncoderFaultMinVoltage = 1.0;
    public static final double kCalibrationVoltage = -2.0; // Calibrate downwards

    public static final MotionProfileConstraints kElevatorConstraints = new MotionProfileConstraints(3.0, 3.0);
    public static final double kElevatorMaxHeight = 2.0;
    public static final double kElevatorVoltageCap = 12.0;
    public static final double kElevatorHallEffectPosition = 0.1;

    public static final MotionProfileConstraints kWristConstraints = new MotionProfileConstraints(3.0, 3.0);
    public static final double kWristMaxAngle = 180.0;
    public static final double kWristVoltageCap = 12.0;
    public static final double kWristHallEffectPosition = 0.0;



}
