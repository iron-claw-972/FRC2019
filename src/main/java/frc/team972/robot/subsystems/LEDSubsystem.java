package frc.team972.robot.subsystems;

import frc.team972.robot.Constants;
import frc.team972.robot.loops.ILooper;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDSubsystem extends Subsystem {
    private static LEDSubsystem mInstance = new LEDSubsystem();

    private DigitalOutput mLED;


    public LEDSubsystem() {
        mLED = new DigitalOutput(Constants.kLEDId);
        mLED.set(true);

        new Thread(new Runnable() {
            public void run() {
                while(true) {
                    mLED.set(true);
                    sleep(1000);
                    mLED.set(false);
                    sleep(1000);
                }
            }

            private void sleep(int millis) {
                try{
                    Thread.sleep(millis);
                } catch(Exception e) {

                }
            }

        }).start();
    }

    public void writeToLog() {
    }

    public void fastPeriodic() {

    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {

    }

    public void stop() {
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public static LEDSubsystem getInstance() {
        return mInstance;
    }
}