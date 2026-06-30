// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Hopper;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  public ShootCommand() {
    // Runs on command creation (usually robot boot)
  }

  @Override
  public void initialize() {
    HopperSubsystem.setSpinnerMotor(8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HopperSubsystem.setKickerVelocity(10);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HopperSubsystem.setSpinnerMotor(0);
    HopperSubsystem.setKickerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
