// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMoveCommand extends Command {
  boolean intakeDeploy;

  /*When the intakeDeploy parameter is set to true, intake will deploy to max
    Otherwise (false) intake is set to the min position*/
  public IntakeMoveCommand(boolean intakeDeploy) {
    this.intakeDeploy = intakeDeploy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeDeploy) {
      if(DriverStation.isAutonomous()){
        IntakeSubsystem.setIntakeDeployPosition(IntakeConstants.IntakeOutPosition + .3);
      }
      else{
      IntakeSubsystem.setIntakeDeployPosition(IntakeConstants.IntakeOutPosition);
    }
  }
    else
    {
      IntakeSubsystem.setIntakeDeployPosition(IntakeConstants.IntakeDeploySoftMin);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}