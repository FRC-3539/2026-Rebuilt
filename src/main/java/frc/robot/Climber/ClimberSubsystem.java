// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Climber;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IDConstants;

public class ClimberSubsystem extends SubsystemBase {
  private static TalonFX climberDeployMotor, climberMotor;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    // climberDeployMotor = new TalonFX(IDConstants.ClimberDeployMotorID, "rio");
    // climberDeployMotor.getConfigurator()
    // .apply(new
    // TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    climberMotor = new TalonFX(IDConstants.ClimberMotorID, "CANivore");
    climberMotor.getConfigurator()
        .apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
    climberMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(ClimberConstants.ClimberP)
        .withKI(ClimberConstants.ClimberI)
        .withKD(ClimberConstants.ClimberD)
        .withKV(ClimberConstants.ClimberV));
    climberMotor.getConfigurator()
        .apply(new MotionMagicConfigs().withMotionMagicAcceleration(ClimberConstants.ClimberAcceleration)
            .withMotionMagicCruiseVelocity(ClimberConstants.ClimberCruiseVelocity));
  }

  public static void setClimberDeployMotor(double voltage) {
    climberDeployMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void setClimberMotor(double voltage) {
    climberMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void zeroClimberMotor() {
    climberMotor.setPosition(0);
  }

  public static void setClimberPosition(double requestedClimberPosition){
    climberMotor.setControl(new MotionMagicVoltage(requestedClimberPosition));
  }
 public static double getClimberPosition(){
  return climberMotor.getPosition().getValueAsDouble();

 }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
