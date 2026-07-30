// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AdaptiveAutos;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RngPathCommand extends Command {
private Debouncer cameraCheck;
private Debouncer targetCheck;
 private Command path;
 PathPlannerPath PPpath;
  /** Creates a new RngPathCommand. */
  public RngPathCommand() {

    
    try {

      PPpath = PathPlannerPath.fromPathFile("RngPath");
          

    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
    cameraCheck = new Debouncer(5);
    targetCheck = new Debouncer(5);
    path = AutoBuilder.followPath(PPpath);
    path.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      boolean hasTarget = VisionSubsystem.resultBackLeft.isPresent()
      || VisionSubsystem.resultLeft.isPresent()
      || VisionSubsystem.resultBackRight.isPresent()
      || VisionSubsystem.resultRight.isPresent();



      boolean missingACamera = 
        !VisionSubsystem.leftCam.isConnected() 
        || !VisionSubsystem.backLeftCam.isConnected() 
        || !VisionSubsystem.rightCam.isConnected() 
        || !VisionSubsystem.backRightCam.isConnected();
      if (cameraCheck.calculate(missingACamera) || targetCheck.calculate(!hasTarget)) 
      {
       path.cancel();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return path.isFinished();
  }
}
