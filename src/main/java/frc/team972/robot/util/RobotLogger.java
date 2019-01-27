package frc.team972.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotLogger {

	public static void print(String message) {
		System.out.println("[ironclaw_logs][norm]: " + message);

	}

	public static void warning(String message) {
		DriverStation.reportWarning("[ironclaw_logs][WARNING]: " + message, false);
	}

	public static void urgentError(String message) {
		System.out.println("**URGENT MESSAGE**");
		DriverStation.reportError("**URGENT** [ironclaw_logs] **URGENT** : " + message, false);
		System.out.println("**URGENT MESSAGE**");
	}

}
