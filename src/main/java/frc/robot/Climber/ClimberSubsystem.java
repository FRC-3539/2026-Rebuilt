// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  private static TalonFX climberDeployMotor, climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climberDeployMotor = new TalonFX(0, "rio");
    climberDeployMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    climberMotor = new TalonFX(0, "rio");
    climberMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
  }

  public static void setClimberDeployMotor(double voltage) {
    climberDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setclimberMotor(double voltage) {
    climberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
