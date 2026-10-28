// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.TeleOpAuton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopAutonCommand extends Command {
  /** Creates a new LeftScriptCommand. 
     * @return */
    
  Command teleopauto;
  
   public TeleopAutonCommand() {
    }
      

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    teleopauto = new PathPlannerAuto("TestAuton"); 
    teleopauto.schedule();
    System.out.println("Chad boy koz");
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     System.out.println("Command is running");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   teleopauto.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return teleopauto.isFinished()
      || !MathUtil.isNear(0, RobotContainer.driverController.getLeftTriggerAxis(), 0.1)
      || !MathUtil.isNear(0, RobotContainer.driverController.getRightTriggerAxis(), 0.1);
  }
}
