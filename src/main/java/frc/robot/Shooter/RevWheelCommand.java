// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import frc.robot.Intake.ShooterSubsystem;
import frc.robot.Utilities.Utils;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RevWheelCommand extends Command {
  /** Creates a new RevWheelCommand. */
  public RevWheelCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     ShooterSubsystem.setShootVelocity(Utils.ssToAngularVelocity(0.0,1).baseUnitMagnitude());

//  ShooterSubsystem.setShootVelocity(Utils.ssToAngularVelocity(ShooterConstants.shootDiameter, ShooterConstants.shootSS).baseUnitMagnitude());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.setShootVelocity(0);
    ShooterSubsystem.setFeedBeltVelocity(Utils.ssToAngularVelocity(IntakeConstants.feedBeltDiameter, IntakeConstants.feedBeltSS).baseUnitMagnitude());
    ShooterSubsystem.setFeedWheelVelocity(Utils.ssToAngularVelocity(IntakeConstants.feedWheelDiameter, IntakeConstants.feedWheelSS).baseUnitMagnitude());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
