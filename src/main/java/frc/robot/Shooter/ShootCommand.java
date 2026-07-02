// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Hopper.HopperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  public ShootCommand() {
    // Runs on command creation (usually robot boot)
  }

  @Override
  public void initialize() {
    ShooterSubsystem.revState = true;
    ShooterSubsystem.raiseHoodState = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HopperSubsystem.setKickerVelocity(10);
    if (ShooterSubsystem.turretReady()) {
      HopperSubsystem.setSpinnerMotor(8);
    } else {
      HopperSubsystem.setSpinnerMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HopperSubsystem.setSpinnerMotor(0);
    HopperSubsystem.setKickerVelocity(0);
    ShooterSubsystem.revState = false;
    ShooterSubsystem.raiseHoodState = false;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
