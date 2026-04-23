// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.Hopper.HopperSubsystem;
import frc.robot.LED.LEDSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class ShootCommand extends Command {
  /** Creates a new ShootCommand. */
  // private Timer timer = new Timer();
  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.restart();
    ShooterSubsystem.raiseHoodState = true;
    LEDSubsystem.shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterSubsystem.turretReady()) { // && timer.get() < 1.5) {
      HopperSubsystem.setKickerVelocity(ShooterSubsystem.getShooterVelocity() * 4 / 2.25);
      HopperSubsystem.setSpinnerMotor(HopperConstants.SpinnerVoltage);

    }
    // else if (timer.get() < 1.75){
    //   HopperSubsystem.setKickerVelocity(-10);
    //   HopperSubsystem.setSpinnerMotor(-.5 * HopperConstants.SpinnerVoltage);
    // }
    // else if (timer.get() > 1.75){
    //   timer.restart();
    // }
    else {
      HopperSubsystem.setKickerVoltage(0);
      HopperSubsystem.setSpinnerMotor(0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    HopperSubsystem.setKickerVoltage(0);
    HopperSubsystem.setSpinnerMotor(0);
    ShooterSubsystem.raiseHoodState = false;
    LEDSubsystem.shooting = false;
    // timer.stop(); 
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
