package frc.team972.robot.util;

public class CoordinateDriveSignal {
    double x;
    double y;
    double rotation;
    boolean noFieldOrient;

    public CoordinateDriveSignal(double x, double y, double rotation, boolean noFieldOrient) {
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.noFieldOrient = noFieldOrient;
    }

    public void addRotation(double a) {
        rotation = rotation + a;
    }

    public void setRotation(double a) {
        rotation = a;
    }

    public double getRotation() {
        return rotation;
    }

    public boolean getFieldOrient() {
        return !noFieldOrient;
    }

    public String toString() {
        return ("x: " + x + " y: " + y + " rot: " + rotation);
    }
}
