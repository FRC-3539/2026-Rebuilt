// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimberPositionCommand extends Command {

  double requestedClimberPosition;
  Timer timer = new Timer();
  /** Creates a new SetClimberCommand. */
  public SetClimberPositionCommand(double requestedClimberPosition) {
    addRequirements(RobotContainer.climberSubsystem);
    this.requestedClimberPosition = requestedClimberPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ClimberSubsystem.setClimberPosition(requestedClimberPosition);
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
    if(MathUtil.isNear(requestedClimberPosition, ClimberSubsystem.getClimberPosition(), 2)){
      timer.start();
    }
    else{ timer.reset();

    }
    return timer.advanceIfElapsed(1);
    // return true;

  }
}
