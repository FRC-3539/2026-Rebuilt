// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static TalonFX intakeMotor, intakeDeployMotor;
  private static double requestedIntakeDeployPos;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(0, "rio");
    intakeMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive));

    intakeDeployMotor = new TalonFX(0, "rio");
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
  }

  public static double getIntakeDeployAngle() {
    return intakeDeployMotor.getPosition().getValueAsDouble();
  }

  public static void setIntakeDeployAngle(double angle) {
    requestedIntakeDeployPos = angle;
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
