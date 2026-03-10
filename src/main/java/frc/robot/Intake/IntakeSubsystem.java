// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static TalonFX intakeMotor, intakeDeployMotor;
  private static double requestedIntakeDeploy;
  public static boolean autonPulse = false;

  private Command intakePulse = new IntakePulseCommand();

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IDConstants.IntakeMotorID, "CANivore");
    intakeDeployMotor = new TalonFX(IDConstants.IntakeDeployMotorID, "CANivore");

    intakeMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(120)
            .withSupplyCurrentLowerLimit(60)
            .withSupplyCurrentLowerTime(2));
    intakeMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));
    intakeDeployMotor.getConfigurator().apply(
        new TalonFXConfiguration().MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive));

    intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeDeployMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(60).withSupplyCurrentLowerTime(2));
    intakeDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(IntakeConstants.IntakeDeploySoftMax)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(IntakeConstants.IntakeDeploySoftMin));
    intakeDeployMotor.getConfigurator().apply(new SlotConfigs()
        .withKP(IntakeConstants.IntakeDeployP)
        .withKI(IntakeConstants.IntakeDeployI)
        .withKD(IntakeConstants.IntakeDeployD));
    intakeDeployMotor.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicAcceleration(IntakeConstants.IntakeDeployAcceleration)
        .withMotionMagicCruiseVelocity(IntakeConstants.IntakeDeployCruiseVelocity));

    // intakeDeployMotor.getConfigurator().apply(new
    // SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
    // .withForwardSoftLimitThreshold(
    // (IntakeConstants.IntakeDeploySoftMax -
    // IntakeConstants.IntakeDeployHomePositionOffset)
    // / IntakeConstants.IntakeDeployInchesPerRotation)
    // .withReverseSoftLimitEnable(true)
    // .withReverseSoftLimitThreshold(
    // (IntakeConstants.IntakeDeploySoftMin -
    // IntakeConstants.IntakeDeployHomePositionOffset)
    // / IntakeConstants.IntakeDeployInchesPerRotation));

  }

  public static void setIntakeDeployPosition(double request) {
    requestedIntakeDeploy = request;
  }

  public static void zeroIntakeDeployMotor() {
    intakeDeployMotor.setPosition(0);
  }

  public static double getIntakeDeployRotations() {
    return intakeDeployMotor.getPosition().getValueAsDouble();
  }

  public static void setIntakeMotor(double voltage) {
    intakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(true));
  }

  public static void log() {
    // Slider Position
    SmartDashboard.putNumber("/Intake/Position", getIntakeDeployRotations());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if ((RobotContainer.operatorController.rightTrigger().getAsBoolean()
          && !RobotContainer.operatorLeftStickUp.getAsBoolean()
          && !intakePulse.isScheduled()
          && requestedIntakeDeploy > 2
          && RobotContainer.driveSubsystem.getPose2d().getX() < 5.3)
        || autonPulse) {
      intakePulse.schedule();
    } else if ((!RobotContainer.operatorController.rightTrigger().getAsBoolean()
        || RobotContainer.operatorLeftStickUp.getAsBoolean()) && intakePulse.isScheduled()) {
      intakePulse.cancel();
    }
    if(!DriverStation.isTest())
    {
    intakeDeployMotor.setControl(new MotionMagicVoltage(requestedIntakeDeploy));

    }
    else {
        intakeDeployMotor.setControl(new VoltageOut(0));
    }
    log();
  }
}
