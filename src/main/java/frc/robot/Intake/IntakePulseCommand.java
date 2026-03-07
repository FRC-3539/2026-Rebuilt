// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePulseCommand extends SequentialCommandGroup {
  /** Creates a new IntakePulseCommand. */
  public IntakePulseCommand() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetIntakeCommand(IntakeConstants.IntakeDeploySoftMax), new WaitCommand(1.5),
        new SetIntakeCommand(IntakeConstants.IntakeDeploySoftMin), new WaitCommand(0.3));

  }
}