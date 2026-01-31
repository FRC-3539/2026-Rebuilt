// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static TalonFX intakeMotor, intakeDeployMotor;
  private static double requestedIntakeDeployPos;
  public static double requestedIntakePos = 0;

  public IntakeSubsystem() {
    intakeMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));
    intakeDeployMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(60).withSupplyCurrentLowerTime(2));        
    intakeMotor = new TalonFX(IDConstants.IntakeMotorID, "rio");
    intakeMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));
    intakeDeployMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));
    intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeDeployMotor = new TalonFX(IDConstants.IntakeDeployMotorID, "rio");
    intakeDeployMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));
    intakeDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0));
    intakeDeployMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(0)
        .withKI(0)
        .withKD(0));

        intakeDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(
            (IntakeConstants.IntakeDeploySoftMax - IntakeConstants.IntakeDeployHomePositionOffset)
                / IntakeConstants.IntakeDeployInchesPerRotation)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(
            (IntakeConstants.IntakeDeploySoftMin - IntakeConstants.IntakeDeployHomePositionOffset)
                / IntakeConstants.IntakeDeployInchesPerRotation));

        
  }

  public static double getIntakeDeployAngle() {
    return intakeDeployMotor.getPosition().getValueAsDouble();
  }

  public static void setIntakeDeployPosition(double request) {
    requestedIntakePos = request;
  }

  public static double degreesToIntakeDeployRotations(double degrees) {
    return Units.degreesToRotations(degrees);
  }

  public static void setIntakeMotor(double voltage)
  {
    intakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeDeployMotor.setControl(new MotionMagicVoltage(degreesToIntakeDeployRotations(requestedIntakeDeployPos)));
  }
}
