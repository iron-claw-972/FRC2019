package frc.team972.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.team972.robot.loops.ILooper;
import frc.team972.robot.driver_utils.TalonSRXFactory;
import frc.team972.robot.util.PIDHandler;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.team972.robot.Constants;

public class Arm extends Subsystem {
    //General initializations
    private static Arm mArm = new Arm();
    private static boolean everythingStopped = false;
    
    //Arm motor and encoder initializations
    public static double desiredArmAngle = 0;
    private static double currentArmAngle = 0;
    public static final double clicksToDegrees = 1;
    private static Encoder pArmEncoder = new Encoder(Constants.mArmEncoderChannelA, Constants.mArmEncoderChannelB, false, Encoder.EncodingType.k4X);
    private static final double[] angleArray = new double[] {20,30,40,50}; //different set heights

    //PID initializations
    private final double kProportion = 0;
    private final double kIntegral = 0;
    private final double kDerivative = 0;
    private PIDHandler PIDControl = new PIDHandler(angleArray[angleArray.length-1], kProportion, kIntegral, kDerivative, 0);

    public static TalonSRX armTalon = TalonSRXFactory.createDefaultTalon(Constants.mArmTalonId);

    public Arm (){
        zeroSensors();
        pArmEncoder.setDistancePerPulse(clicksToDegrees);
    }

    public double getCurrentArmAngle(){
        return currentArmAngle;
    }

    public void writeToLog() {
    }

    public void readPeriodicInputs() {
    }

    public void writePeriodicOutputs() {

        if (!everythingStopped){
            currentArmAngle = pArmEncoder.get();
            PIDControl.setDesired(desiredArmAngle);
            armTalon.set(ControlMode.PercentOutput, PIDControl.output(currentArmAngle));
        }
        //System.out.println("Test");
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        System.out.println("Current Angle: " + currentArmAngle + " | Desired Arm Angle: " + desiredArmAngle);
    }

    public void stop() {
        everythingStopped = true;
    }

    public void start() {
        everythingStopped = false;
    }

    public void zeroSensors() {
        pArmEncoder.reset();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public void setDesiredArmAngle(double angle){
        desiredArmAngle = angle;
    }
    
    public void setDesiredArmLevel(int level){
        desiredArmAngle = angleArray[level];
    }

    public double getDesiredArmAngle(){
        return desiredArmAngle;
    }

    public static Arm getInstance() {
        return mArm;
    }
}
