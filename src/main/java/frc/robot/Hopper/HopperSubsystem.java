// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Hopper;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IDConstants;

public class HopperSubsystem extends SubsystemBase {
  // Create (declare) objects here

  private static TalonFX spinnerMotor, kickerMotor;
  static VelocityVoltage velocityVoltageControl = new VelocityVoltage(0);

  public HopperSubsystem() {
    // spinnerMotor = new TalonFX(IDConstants.SpinnerMotorID, canbus:"CANIVORE");
     spinnerMotor = new TalonFX(IDConstants.SpinnerMotorID, "CANivore");
    spinnerMotor.setNeutralMode(NeutralModeValue.Brake);
    spinnerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    spinnerMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

    kickerMotor = new TalonFX(IDConstants.KickerMotorID, "rio");
    kickerMotor.setNeutralMode(NeutralModeValue.Brake);
    kickerMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    kickerMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));
    kickerMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(HopperConstants.KickerP)
        .withKI(HopperConstants.KickerI)
        .withKD(HopperConstants.KickerD)
        .withKV(HopperConstants.KickerV));
  }
  

  public static void setSpinnerMotor(double voltage) {
    spinnerMotor.setControl(new VoltageOut(voltage));
  }

  public static void setKickerVelocity(double velocity) {
    kickerMotor.setControl(velocityVoltageControl.withVelocity(velocity));
  }

  public static double getSpinnerCurrent() {
    return spinnerMotor.getSupplyCurrent().getValueAsDouble();
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
//tyler = thao;