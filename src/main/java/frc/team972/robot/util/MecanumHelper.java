package frc.team972.robot.util;

public class MecanumHelper {

    public static CoordinateDriveSignal mecanumDrive(double x, double y, double rotation, boolean noFieldOrient) {
        return new CoordinateDriveSignal(handleDeadband(x, 0.05), handleDeadband(y, 0.05), handleDeadband(rotation, 0.05), noFieldOrient);
    }

    /*
        This is only used by classes that need to calculate the necessary Mecanum transforms!
        Just a fun math class.
        TODO: Profile performance. Lots of recycling can be done.
     */
    public static DriveSignal cartesianCalculate(CoordinateDriveSignal signal, double current_angle) {
        double[] rotatedVector = rotateVector(signal.x, signal.y, current_angle);

        signal.x = rotatedVector[0];
        signal.y = rotatedVector[1];

        DriveSignal driveSignal = new DriveSignal(
                -signal.x + signal.y - signal.rotation,
                -signal.x - signal.y - signal.rotation,
                signal.x + signal.y - signal.rotation,
                signal.x - signal.y - signal.rotation
        );

        normalize(driveSignal);

        return driveSignal;
    }

    public static void normalize(DriveSignal driveSignal) {
        double mag[] = new double[]{driveSignal.getLeftFront(), driveSignal.getLeftBack(), driveSignal.getRightFront(), driveSignal.getRightBack()};
        double maxMag = 0;
        for (int i = 1; i < mag.length; i++) {
            double temp = Math.abs(mag[i]);
            if (maxMag < temp) {
                maxMag = temp;
            }
        }

        if(maxMag > 1.0) {
            driveSignal.set(driveSignal.getLeftFront() / maxMag, driveSignal.getLeftBack() / maxMag, driveSignal.getRightFront() / maxMag, driveSignal.getRightBack() / maxMag);
        }
    }

    private static double[] rotateVector(double x, double y, double angle) {
        double angleRad = Math.toRadians(angle);
        double cosA = Math.cos(angleRad);
        double sinA = Math.sin(angleRad);

        return new double[]{(x * cosA) - (y * sinA), (x * sinA) + (y * cosA)};
    }

    public static double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}