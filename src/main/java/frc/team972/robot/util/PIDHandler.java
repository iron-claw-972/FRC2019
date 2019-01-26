package frc.team972.robot.util;

public class PIDHandler {
    private double desired;
    private double KP; //Multipliers
    private double KI;
    private double KD;
    private double bias;

    private double integral;
    private double errorPrior;
    private long lastCheckMillis;

    public PIDHandler(double desired, double KP, double KI, double KD, double bias) {
        this.desired = desired;
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.bias = bias;

        this.integral = 0;
        this.errorPrior = 0;
        this.lastCheckMillis = System.currentTimeMillis();
    }

    public double output(double actual) {
        double error = desired - actual;
        long iterationTime = System.currentTimeMillis() - lastCheckMillis;
        integral += error*iterationTime;
        double derivative = (error-errorPrior)/iterationTime;
        errorPrior = error;
        return KP*error + KI*integral + KD*derivative + bias;
    }

    public void setDesired(double desired) {
        this.desired = desired;
    }
}
