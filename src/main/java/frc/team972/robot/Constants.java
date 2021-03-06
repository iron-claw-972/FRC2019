package frc.team972.robot;

import frc.team972.robot.controls.MotionProfileConstraints;
import frc.team972.robot.statemachines.SuperstructureStateMachine;

public class Constants {

    public static final double dt = 0.01;

    // CONTROLLER PORT IDS
    public static final int kDriveGamepadPort = 1;

    public static final double kStickToVelocity = 40;
    public static final double kMecanumRotationK1 = 0.00125;
    public static final double kMecanumRotationK2 = 0.000125;
    public static final double kMecanumRotationK3 = 1.05;


    // DRIVE MOTOR IDS
    // TODO: Fill with real Talon ID

    public static final double kDriveWheelRadius = 3.0;
    public static final double kDriveVoltageCap = 6.0;
    public static final double kDriveVelocityCap = 5.0;
    public static final double kDriveTicksPerRev = 4096;

    // FOR OLD VELOCITY CONTROLLERS. HAS BEEN REPLACED BY FULL-STATE LQR CONTROLLERS
    public static final double kDriveVelocityFF = 0.125; // dutyCycle per revs
    public static final double kDriveVelocityPGain = 0.095;

    public static final int kLeftFrontId = 1;
    public static final int kLeftBackId = 3;
    public static final int kRightFrontId = 2;
    public static final int kRightBackId = 4;

    //HATCH PISTON ID
    public static final int kHatchIntakePistonChannelAId = 2;
    public static final int kHatchIntakePistonChannelBId = 3;

    //ELEVATOR MOTOR IDS
    public static final int kElevatorMotorId = 8;
    public static final int kElevatorSlaveAMotorId = 6;
    public static final int kElevatorSlaveBMotorId = 7;
    public static final int kElevatorSlaveCMotorId = 5;

    //WRIST MOTOR IDS
    public static final int kWristMotorId = 9;
    public static final int kWristRollerMotorId = 10;

    public static final int kLongCANTimeoutMs = 100;

    //STATE SPACE MAGICAL STUFF -- DO NOT TOUCH
    public static final int kEncoderFaultTicksAllowed = 1024;
    public static final double kEncoderFaultMinVoltage = 1.0;
    public static final double kCalibrationVoltage = -0.5; // Calibrate downwards

    public static final MotionProfileConstraints kElevatorConstraints = new MotionProfileConstraints(4.0, 10.0);
    public static final double kElevatorMaxHeight = 2.0;
    public static final double kElevatorVoltageCap = 8.0;
    public static final int kElevatorEncoderCountPerRev = 1024;
    public static final double kElevatorHallEffectPosition = 0.0;
    public static final double kElevatorSpoolDiameter = 1.5 * 0.0254;

    public static final MotionProfileConstraints kWristConstraints = new MotionProfileConstraints(3.0, 4.0);
    public static final double kWristMaxAngle = Math.PI;
    public static final double kWristVoltageCap = 3.0;
    public static final double kWristHallEffectPosition = Math.toRadians(SuperstructureStateMachine.kWristStartOffset);
    public static final int kWristEncoderCountPerRev = 1024;

}
