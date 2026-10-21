// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

import org.frcteam3539.Byte_Swerve_Lib.control.MaxAccelerationConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.MaxVelocityConstraint;
import org.frcteam3539.Byte_Swerve_Lib.control.SimplePathBuilder;
import org.frcteam3539.Byte_Swerve_Lib.control.Trajectory;
import org.frcteam3539.Byte_Swerve_Lib.control.TrajectoryConstraint;

public class BBAutoAlignCommand extends Command {
	/** Wrapper command to generate a trajectory to the nearest Stage Pose */
	Pose2d point;

	public BBAutoAlignCommand(Pose2d point) {
		this.point = point;
	}

	// Called when the command is initially scheduled.
	@Override

	public void initialize() {
		Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();
		RobotContainer.driveSubsystem.getFollower()
					.follow(new Trajectory(new SimplePathBuilder(robotPose).lineTo(point).build(),
							new TrajectoryConstraint[] { (TrajectoryConstraint) new MaxAccelerationConstraint(1),
									(TrajectoryConstraint) new MaxVelocityConstraint(2) },
							.05));

		DriveSubsystem.publishPose2d("TargetPoint", point);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		// Nothing to do here
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		RobotContainer.driveSubsystem.getFollower().cancel();
		if (!interrupted) {
			new ParallelDeadlineGroup(new WaitCommand(0.25), new RumbleCommand()).schedule();
		}
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return RobotContainer.driveSubsystem.getFollower().getCurrentTrajectory().isEmpty();
	}
}