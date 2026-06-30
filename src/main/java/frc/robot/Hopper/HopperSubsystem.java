// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Hopper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  // Create (declare) objects here

  public HopperSubsystem() {
    // Set up (initialize) objects here
  }

  @Override
  public void periodic() {
    // Runs every code loop
  }
}

// public static void log() {
//   SmartDashboard.putNumber("/Hopper/KickerTemp", kickerMotor.getDeviceTemp().getValueAsDouble());
//   SmartDashboard.putNumber("/Hopper/SpinnerTemp", spinnerMotor.getDeviceTemp().getValueAsDouble());
// }