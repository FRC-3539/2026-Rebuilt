// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  	// Create (declare) objects here
	private static Timer timer = new Timer();
	private static double intakeDeployPosition;
	private static TalonFX intakeMotor, intakeDeployMotor;
	static VelocityVoltage velocityVoltageControl = new  VelocityVoltage(0);
	public IntakeSubsystem() {
    	// Set up (initialize) objects here
		intakeMotor = new TalonFX(IDConstants.IntakeMotorID, "CANivore");
		intakeMotor.setNeutralMode(NeutralModeValue.Brake);
		intakeMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    	intakeMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(70).withSupplyCurrentLowerLimit(45).withSupplyCurrentLowerTime(2));

		intakeDeployMotor = new TalonFX(IDConstants.IntakeDeployMotorID, "CANivore");
		intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);
		intakeDeployMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    	intakeDeployMotor.getConfigurator()
        .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40).withSupplyCurrentLowerLimit(40).withSupplyCurrentLowerTime(2));
		intakeDeployMotor.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitEnable(true)
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
	}

	public static void setIntakeMotor(double voltage) {
		intakeMotor.setControl( new VoltageOut(voltage));
	}

	public static double getIntakeCurrent() {
		return intakeMotor.getSupplyCurrent().getValueAsDouble();
	}

	public static void setIntakeDeploy(double Position) {
		intakeDeployPosition = Position;
	}

	@Override
	public void periodic() {
		intakeDeployMotor.setControl(new MotionMagicVoltage(intakeDeployPosition));
	}
}








		/* 
		if (DriverStation.isTest()) {
			intakeDeployMotor.setControl(new VoltageOut(0));
			setIntakeVoltage(0);
			return;
		}

		double intakePosition = requestedIntakeDeploy;
		boolean shouldPulse = 
			// (RobotContainer.operatorController.rightTrigger().getAsBoolean()
			// 	&& !RobotContainer.operatorLeftStickUp.getAsBoolean()
			// 	&& RobotContainer.driveSubsystem.getPose2d().getX() < 5.3
			// 	&& requestedIntakeDeploy > 1) ||
				autonPulse;

		if (timer.get() < 1) {
			// Out
			if (shouldPulse)
				intakePosition = IntakeConstants.IntakeOutPosition;
		}

		else if (timer.get() < 1.5) {
			// In
			if (shouldPulse)
				intakePosition = IntakeConstants.IntakeOutPosition * 0.4;

		} else timer.restart();

		if (getIntakeDeployRotations() > IntakeConstants.IntakeOutPosition - .4 && DriverStation.isAutonomous()
				&& intakePosition == IntakeConstants.IntakeOutPosition) {
			intakeDeployMotor.setControl(new TorqueCurrentFOC(35));
		} else {
			intakeDeployMotor.setControl(new MotionMagicVoltage(intakePosition));
		}
		*/