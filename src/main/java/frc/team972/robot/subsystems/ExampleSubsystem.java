package frc.team972.robot.subsystems;

public class ExampleSubsystem extends Subsystem {
    private static ExampleSubsystem mInstance = new ExampleSubsystem();

    private ExampleSensor exampleSensor = new ExampleSensor();
    private ExampleMotor exampleMotor = new ExampleMotor();

    private double desiredVoltage = 0.0; //Initialize to sane values

    public void writeToLog() {
    }

    public double getExampleMotorVoltage() {
        return exampleMotor.getVoltage_();
    }

    public void fastPeriodic() {
        //Handle all control-loop logic here
        if(desiredVoltage > 12.0) {
            desiredVoltage = 12.0;
        } else if(desiredVoltage < -12.0) {
            desiredVoltage = -12.0;
        }

        //Read all sensors and perform logic associated with control
        double real_voltage = desiredVoltage * 0.5;
        if(exampleSensor.motorStop()) {
            real_voltage = 0.0;
        }
        System.out.print(""); // We have to do this to keep the unit test thread alive because JUnit likes to kill spawned threads....

        //Output control-loop calculated values to hardware
        exampleMotor.set(real_voltage);
    }

    public void setDesiredVoltage(double desiredVoltage) {
        this.desiredVoltage = desiredVoltage;
    }

    public boolean checkSystem() {
        return true;
    }

    public void outputTelemetry() {
        System.out.println("The current example motor voltage is " + getExampleMotorVoltage());
    }

    public void stop() {
    }

    public void zeroSensors() {
    }

    public static ExampleSubsystem getInstance() {
        return mInstance;
    }
}

class ExampleSensor {
    public boolean motorStop() {
        return false;
    }
}

class ExampleMotor {
    public double getVoltage_() {
        return voltage_;
    }

    double voltage_ = 0;
    public void set(double voltage) {
        voltage_ = voltage;
    }

}