// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.frcteam3539.Byte_Swerve_Lib.control.PidConstants;
import org.frcteam3539.Byte_Swerve_Lib.control.PidController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;


public class DriveCommand extends Command {
	/** Creates a new DriveCommand. */
	private PidController rotationController;

	double maxVelocity = RobotContainer.driveSubsystem.maxVelocity;

	double maxRotationalVelocity = RobotContainer.driveSubsystem.maxRotationalVelocity;

	double rotationDeadband = maxRotationalVelocity * 0.02;
	private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
			.withDeadband(maxVelocity * 0.02).withRotationalDeadband(rotationDeadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

	public DriveCommand() {
		addRequirements(RobotContainer.driveSubsystem);

		rotationController = new PidController(new PidConstants(DriveConstants.AlignP, 0, 0));

		rotationController.setInputRange(-Math.PI, Math.PI);
		rotationController.setOutputRange(-1, 1);
		rotationController.setContinuous(true);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		SwerveRequest request = new SwerveRequest.Idle();

		double speedMultiplier = DriveConstants.SpeedMultiplier;
		double rotationSpeedMultiplier = DriveConstants.RotationSpeedMultiplier;

		if (RobotContainer.rightDriverTrigger.getAsBoolean()) // Slow Mode
		{
			speedMultiplier = DriveConstants.TurboSpeedMultiplier;
			rotationSpeedMultiplier = DriveConstants.TurboRotationSpeedMultiplier;
		}

		if (RobotContainer.rightDriverBumper.getAsBoolean()) { // Robot Centric
			request = driveRobotCentric
					.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity * speedMultiplier)
					.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity * speedMultiplier)
					.withRotationalRate(-RobotContainer.driverController.getRightX() * maxRotationalVelocity
							* rotationSpeedMultiplier)
					.withRotationalDeadband(rotationDeadband);
		} else {
			request = driveFieldCentric
					.withVelocityX(-RobotContainer.driverController.getLeftY() * maxVelocity * speedMultiplier)
					.withVelocityY(-RobotContainer.driverController.getLeftX() * maxVelocity * speedMultiplier)
					.withRotationalRate(-RobotContainer.driverController.getRightX() * maxRotationalVelocity
							* rotationSpeedMultiplier)
					.withRotationalDeadband(rotationDeadband);
		}

		RobotContainer.driveSubsystem.applyRequest(request);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

	private static Translation2d modifyJoystick(Translation2d joystick) {
		// Deadband
		Rotation2d rotation = joystick.getAngle();
		double distance = joystick.getNorm();

		double distanceModified = Math.copySign(Math.pow(distance, 3), distance);

		return new Translation2d(distanceModified, rotation);
	}
}
