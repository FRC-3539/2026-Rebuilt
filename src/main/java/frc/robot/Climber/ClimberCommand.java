// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {
  /** Creates a new ClimbMotorCommand. */
  public ClimberCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.operatorController.getRightY() < -0.2 || RobotContainer.operatorController.getRightY() > 0.2) {
      ClimberSubsystem
          .setClimberDeployMotor(ClimberConstants.ClimberDeployVoltage * RobotContainer.operatorController.getRightY());
    } else {
      ClimberSubsystem.setClimberDeployMotor(0);
    }

    if (RobotContainer.operatorController.getLeftY() < -0.2 || RobotContainer.operatorController.getLeftY() > 0.2) {
      ClimberSubsystem
          .setClimberMotor(ClimberConstants.ClimberVoltage * RobotContainer.operatorController.getLeftY());
    } else {
      ClimberSubsystem.setClimberMotor(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSubsystem.setClimberDeployMotor(0);
    ClimberSubsystem.setClimberMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
