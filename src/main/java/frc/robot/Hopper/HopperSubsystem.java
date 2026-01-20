// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Hopper;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  private static TalonFX rollerMotor, pushMotor, kickerMotor;

  public HopperSubsystem() {
    rollerMotor = new TalonFX(0, "rio");
    rollerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    pushMotor = new TalonFX(0, "rio");
    pushMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    kickerMotor = new TalonFX(0, "rio");
    kickerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    kickerMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
