package frc.robot.Utilities;

import frc.robot.RobotContainer;
public class LogController {
	// +-----------------------------------+
	// | SET ROBOT LOGGING SETTINGS HERE :> |
	// +-----------------------------------

	public static boolean USE_LOGGING = true;

	static boolean LOG_DRIVE_SUBSYSTEM = true;
	static boolean LOG_INTAKE_SUBSYSTEM = true;
	static boolean LOG_SHOOTER_SUBSYSTEM = true;
	static boolean LOG_VISION_SUBSYSTEM = true;

	static boolean LOG_LEDS = true;

	static boolean SAVE_TO_FILE = false;
	static String LOG_FILE_PATH = "/media/sda1/";
	static boolean USE_NETWORK_TABLES = true;

	// +---------------------------------+
	// | END OF ROBOT LOGGING SETTINGS |
	// +---------------------------------+

	public LogController() {
		if (!USE_LOGGING)
			return;

		// if (Robot.isReal()) {
		// if (SAVE_TO_FILE)
		// Logger.addDataReceiver(new WPILOGWriter(LOG_FILE_PATH));
		// if (USE_NETWORK_TABLES)
		// Logger.addDataReceiver(new NT4Publisher());
		// } else {
		// String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
		// AdvantageScope (or prompt the
		// // user)
		// Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
		// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
		// "_sim"))); // Save outputs to a
		// // new log
		// }

		// logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
		// the
		// "Understanding Data Flow" page
		// Logger.start(); // Start logging! No more data receivers, replay sources, or
		// metadata values may
		// be added.

		// DriverStation.startDataLog(DataLogManager.getLog());
	}

	public void logPeriodic() {
		if (!USE_LOGGING)
			return;
		logDriveSubsystem();
		logIntakeSubsystem();
		logShooterSubsystem();
		// logVisionSubsystem();
	}

	public void logDriveSubsystem() {
		if (!LOG_DRIVE_SUBSYSTEM)
			return;
		// RobotContainer.driveSubsystem.log();
	}

	public void logIntakeSubsystem() {
		if (!LOG_INTAKE_SUBSYSTEM)
			return;
		// IntakeSubsystem.log();
	}

	public void logShooterSubsystem() {
		if (!LOG_SHOOTER_SUBSYSTEM)
			return;
		// ShooterSubsystem.log();
	}
	// public void logVisionSubsystem() {
	// 	if (!LOG_VISION_SUBSYSTEM)
	// 		return;
	// 	RobotContainer.visionSubsystem.log();
	// }
}
