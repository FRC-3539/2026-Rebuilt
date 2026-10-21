// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import java.util.ArrayList;
import java.util.HashSet;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.DriveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDAutoAlignCommand extends Command {
  /** Creates a new PIDAutoAlignCommand. */
  boolean second = false;
  double translateAccelMax = 4;
  double thetaAccelMax = 4;


  ProfiledPIDController xController = new ProfiledPIDController(DriveConstants.TranslationP,
      DriveConstants.TranslationI, DriveConstants.TranslationD, new Constraints(6, translateAccelMax));

  ProfiledPIDController yController = new ProfiledPIDController(DriveConstants.TranslationP,
      DriveConstants.TranslationI, DriveConstants.TranslationD, new Constraints(6, translateAccelMax));

  ProfiledPIDController thetaController = new ProfiledPIDController(DriveConstants.RotationP,
      DriveConstants.RotationI, DriveConstants.RotationD, new Constraints(6, thetaAccelMax));

  Pose2d targetPoint = new Pose2d();

  double maxVelocity = RobotContainer.driveSubsystem.maxVelocity;

  double maxRotationalVelocity = RobotContainer.driveSubsystem.maxRotationalVelocity;

  private final SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric()
      .withDeadband(0).withRotationalDeadband(0) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  public 
  PIDAutoAlignCommand(boolean second) {
    addRequirements(RobotContainer.driveSubsystem);
    this.second = second;


    xController.setTolerance(0.04);
    yController.setTolerance(0.04);
    thetaController.setTolerance(0.035);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double translateAccel = translateAccelMax;
    double thetaAccel = thetaAccelMax;


    xController.reset(RobotContainer.driveSubsystem.getPose2d().getX(), RobotContainer.driveSubsystem.velocityX);
    yController.reset(RobotContainer.driveSubsystem.getPose2d().getY(), RobotContainer.driveSubsystem.velocityY);
    thetaController.reset(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(),
        RobotContainer.driveSubsystem.velocityR);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();

    // Calculate the robot pose after deceleration
    double deceleratedX = robotPose.getX() + RobotContainer.driveSubsystem.velocityX * translateAccel;
    double deceleratedY = robotPose.getY() + RobotContainer.driveSubsystem.velocityY * translateAccel;
    double deceleratedTheta = robotPose.getRotation().getRadians() + RobotContainer.driveSubsystem.velocityR * thetaAccel;

    //robotPose = new Pose2d(deceleratedX, deceleratedY, robotPose.getRotation().plus(new Rotation2d(deceleratedTheta)));
    robotPose = new Pose2d(deceleratedX, deceleratedY, robotPose.getRotation().plus(new Rotation2d(deceleratedTheta)));

    // Log or use the deceleratedPose as needed

    targetPoint = robotPose.nearest(AlignConstants.trenchPoints);
    
    if(!second) return;
    var newPoints = new HashSet<>(AlignConstants.trenchPoints);
    newPoints.remove(targetPoint);
    targetPoint = robotPose.nearest(newPoints);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translateAccel = translateAccelMax;
    double thetaAccel = thetaAccelMax;

    // if(ElevatorSubsystem.requestedElevatorPos>35)
    //   translateAccel = translateAccelElevatorUp;
    //   thetaAccel = thetaAccelElevatorUp;

    SwerveRequest request = new SwerveRequest.Idle();
    var x = xController.calculate(RobotContainer.driveSubsystem.getPose2d().getX(), new State(targetPoint.getX(),0), new Constraints(maxVelocity, translateAccel));
    var y = yController.calculate(RobotContainer.driveSubsystem.getPose2d().getY(), new State(targetPoint.getY(),0), new Constraints(maxVelocity, translateAccel));
    var r = thetaController.calculate(RobotContainer.driveSubsystem.getPose2d().getRotation().getRadians(), new State(targetPoint.getRotation().getRadians(),0), new Constraints(maxVelocity, thetaAccel));
    request = driveFieldCentric
        .withVelocityX(x)
        .withVelocityY(y)
        .withRotationalRate(r);
    RobotContainer.driveSubsystem.applyRequest(request);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // LedSubsystem.setAligning(false);
    // if (!interrupted) {
    //   new ParallelDeadlineGroup(new WaitCommand(0.25), new RumbleCommand()).schedule();
    // }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
