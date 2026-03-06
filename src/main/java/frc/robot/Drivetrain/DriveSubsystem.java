// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.frcteam3539.Byte_Swerve_Lib.control.HolonomicMotionProfiledTrajectoryFollower;
import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.DrivetrainFeedforwardConstants;
import org.frcteam3539.Byte_Swerve_Lib.util.HolonomicFeedforward;
// hi tyler
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Vision.VisionSubsystem;

public class DriveSubsystem extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	/** Creates a new DrivetrainSubsystem. */
	private SwerveRequest swerveRequest = new SwerveRequest.Idle();

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

	public double maxVelocity = 0.0;
	public double maxRotationalVelocity = 0.0;
	public double velocityX, velocityY, velocityR = 0.0;

	public Pigeon2 pigeon = new Pigeon2(IDConstants.PigeonID, "rio");

	private final HolonomicMotionProfiledTrajectoryFollower follower;

	Field2d field = new Field2d();

	public static HashMap<ParentDevice, Alert> connectedDriveAlerts = new HashMap<>();
	public static HashMap<ParentDevice, Alert> wasDisconnectedDriveAlerts = new HashMap<>();

	private Command shortRumbleCommand = new ParallelDeadlineGroup(new WaitCommand(0.5), new RumbleCommand());
	private Command longRumbleCommand = new ParallelDeadlineGroup(new WaitCommand(1.5), new RumbleCommand());

	private void createAlert(ParentDevice device, String deviceName) {
		Alert isAlert = new Alert("Drive Subsystem", deviceName + ": is disconnected", AlertType.kError);
		connectedDriveAlerts.put(device, isAlert);
		Alert wasAlert = new Alert("Drive Subsystem", deviceName + ": was disconnected", AlertType.kError);
		wasDisconnectedDriveAlerts.put(device, wasAlert);
	}

	public DriveSubsystem(SwerveDrivetrainConstants driveTrainConstants,
			SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);

		maxVelocity = modules[0].SpeedAt12Volts;

		Translation2d[] moduleLocations = new Translation2d[modules.length];

		for (int i = 0; i < modules.length; i++) {
			moduleLocations[i] = new Translation2d(modules[i].LocationX, modules[i].LocationY);
			this.getModule(i).getDriveMotor().getConfigurator()
					.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(60)
							.withSupplyCurrentLimitEnable(true)
							.withStatorCurrentLimit(modules[i].SlipCurrent).withStatorCurrentLimitEnable(true));
		}

		double dtRadius = new Translation2d().nearest(Arrays.asList(moduleLocations)).getDistance(new Translation2d());
		maxRotationalVelocity = (maxVelocity / dtRadius);

		configureAutoBuilder();

		DrivetrainFeedforwardConstants FEEDFORWARD_CONSTANTS = new DrivetrainFeedforwardConstants(
				DriveConstants.TranslationV, DriveConstants.TranslationA,
				DriveConstants.TranslationS);

		follower = new HolonomicMotionProfiledTrajectoryFollower(
				new PidConstants(DriveConstants.TranslationP, DriveConstants.TranslationI,
						DriveConstants.TranslationD),
				new PidConstants(DriveConstants.RotationP, DriveConstants.RotationI,
						DriveConstants.RotationD),
				new HolonomicFeedforward(FEEDFORWARD_CONSTANTS));

		SmartDashboard.putData("Field", field);

		createAlert(this.getModule(0).getSteerMotor(), "FrontLeftSteer");
		createAlert(this.getModule(0).getDriveMotor(), "FrontLeftDrive");
		createAlert(this.getModule(1).getSteerMotor(), "FrontRightSteer");
		createAlert(this.getModule(1).getDriveMotor(), "FrontRightDrive");
		createAlert(this.getModule(2).getSteerMotor(), "BackLeftSteer");
		createAlert(this.getModule(2).getDriveMotor(), "BackLeftDrive");
		createAlert(this.getModule(3).getSteerMotor(), "BackRightSteer");
		createAlert(this.getModule(3).getDriveMotor(), "BackRightDrive");

	}

	// public void seedFieldRelative(Trajectory trajectory) {
	// this.seedFieldRelative(trajectory.calculate(0).getPathState().getPose2d());
	// }

	public Pose2d getPose2d() {
		return this.getState().Pose;
	}

	public void applyRequest(SwerveRequest request) {
		this.swerveRequest = request;
	}

	private void configureAutoBuilder() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(
					() -> getState().Pose, // Supplier of current robot pose
					this::resetPose, // Consumer for seeding pose against auto
					() -> getState().Speeds, // Supplier of current robot speeds
					// Consumer of ChassisSpeeds and feedforwards to drive the robot
					(speeds, feedforwards) -> setControl(
							m_pathApplyRobotSpeeds.withSpeeds(speeds)
									.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
									.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
					new PPHolonomicDriveController(
							// PID constants for translation
							new PIDConstants(DriveConstants.TranslationP, DriveConstants.TranslationI,
									DriveConstants.TranslationD),
							// PID constants for rotation
							new PIDConstants(DriveConstants.RotationP, DriveConstants.RotationI,
									DriveConstants.RotationD)),
					config,
					// Assume the path needs to be flipped for Red vs Blue, this is normally the
					// case
					() -> false,
					this // Subsystem for requirements
			);
		} catch (Exception ex) {
			DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
					ex.getStackTrace());
		}
	}

	public static void publishPose2d(String key, Pose2d pose) {
		SmartDashboard.putNumberArray(key, new double[] { pose.getTranslation().getX(), pose.getTranslation().getY(),
				pose.getRotation().getRadians() });
	}

	public HolonomicMotionProfiledTrajectoryFollower getFollower() {
		return follower;
	}

	@Override
	public void periodic() {
		double time = DriverStation.getMatchTime();
		String message = DriverStation.getGameSpecificMessage();
		if (DriverStation.getGameSpecificMessage().length() > 0) {
			switch (message.charAt(0)) {
				// Blue wins auton, red shoorts first
				case 'B':
					if (DriverStation.getAlliance().get() == Alliance.Blue) {
						// Transition Shift Ends
						if (time < 130 && time > 129.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 2 Warning
						if (time < 115 && time > 114.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// Shift 2 Shoot
						if (time < 107 && time > 106.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 2 Ends
						if (time < 80 && time > 79.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// End Warning
						if (time < 65 && time > 64.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// End Shift Starts
						if (time < 57 && time > 56.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
					}

					else {
						// Shift 1 Ends
						if (time < 105 && time > 104.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 3 Warning
						if (time < 90 && time > 89.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// Shift 3 Shoot
						if (time < 82 && time > 81.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 3 Ends
						if (time < 55 && time > 54.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// End Warning
						if (time < 40 && time > 39.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// End Shift Starts
						if (time < 32 && time > 31.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
					}

					break;

				default:
					if (DriverStation.getAlliance().get() == Alliance.Red) {
						// Transition Shift Ends
						if (time < 130 && time > 129.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 2 Warning
						if (time < 115 && time > 114.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// Shift 2 Shoot
						if (time < 107 && time > 106.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 2 Ends
						if (time < 80 && time > 79.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// End Warning
						if (time < 65 && time > 64.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// End Shift Starts
						if (time < 57 && time > 56.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
					}

					else {
						// Shift 1 Ends
						if (time < 105 && time > 104.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 3 Warning
						if (time < 90 && time > 89.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// Shift 3 Shoot
						if (time < 82 && time > 81.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// Shift 3 Ends
						if (time < 55 && time > 54.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
						// End Warning
						if (time < 40 && time > 39.5 && !longRumbleCommand.isScheduled()) {
							longRumbleCommand.schedule();
						}
						// End Shift Starts
						if (time < 32 && time > 31.85 && !shortRumbleCommand.isScheduled()) {
							shortRumbleCommand.schedule();
						}
					}
					break;
			}
		}

		for (ParentDevice device : connectedDriveAlerts.keySet()) {
			Alert isAlert = connectedDriveAlerts.get(device);
			Alert wasAlert = wasDisconnectedDriveAlerts.get(device);

			if (!device.isConnected()) {
				isAlert.set(true);
				wasAlert.set(false);

			} else if (isAlert.get()) {
				isAlert.set(false);
				wasAlert.set(true);

			}
		}

		if (Robot.isSimulation()) {
			updateSimState(0.020, RobotController.getBatteryVoltage());
		}

		field.setRobotPose(getPose2d());

		velocityX = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().Speeds,
				this.getPose2d().getRotation()).vxMetersPerSecond;

		velocityY = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().Speeds,
				this.getPose2d().getRotation()).vyMetersPerSecond;

		velocityR = ChassisSpeeds.fromRobotRelativeSpeeds(this.getState().Speeds,
				this.getPose2d().getRotation()).omegaRadiansPerSecond;

		var robotVelocity = new Translation2d(velocityX, velocityY);
		SmartDashboard.putNumber("/DriveTrain/RobotVelocity", robotVelocity.getNorm());

		SwerveRequest request = new SwerveRequest.Idle();
		request = swerveRequest;

		var driveSignalOpt = follower.update(getPose2d(), Timer.getFPGATimestamp(), Robot.kDefaultPeriod);

		if (follower.getLastState() != null) {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested",
					follower.getLastState().getPathState().getPose2d());
		} else {
			VisionSubsystem.publishPose2d("/DriveTrain/PoseRequested", new Pose2d());
		}

		if (driveSignalOpt.isPresent()) {
			ChassisSpeeds speeds = driveSignalOpt.get();
			request = new SwerveRequest.RobotCentric().withVelocityX(speeds.vxMetersPerSecond)
					.withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond);
		} else {
			request = swerveRequest;
		}

		this.setControl(request);
		// This method will be called once per scheduler run
		if (Robot.isSimulation()) {
			double randomOffset = (Math.random() - 0.5) * 0.2; // Random offset between -0.1 and 0.1
			double simulatedVoltage = RobotController.getBatteryVoltage() + randomOffset;
			SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", simulatedVoltage);
		} else {
			SmartDashboard.putNumber("/DriveTrain/BatteryVoltage", RobotController.getBatteryVoltage());
		}

		publishPose2d("/DriveTrain/Pose", getPose2d());
		SmartDashboard.putNumber("/DriveTrain/Heading", getPose2d().getRotation().getDegrees());

	}
}
