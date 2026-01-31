// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Hopper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class HopperSubsystem extends SubsystemBase {
  private static TalonFX spinnerMotor, kickerMotor;
 
  public HopperSubsystem() {
    spinnerMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));
            kickerMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));
    spinnerMotor = new TalonFX(IDConstants.SpinnerMotorID, "rio");
    spinnerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));

    kickerMotor = new TalonFX(IDConstants.KickerMotorID, "rio");
    kickerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    kickerMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0));
    

  }

   public static void setSpinnerMotor(double voltage)
  {
    spinnerMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  public static void setKickerMotor(double voltage)
  {
    kickerMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
