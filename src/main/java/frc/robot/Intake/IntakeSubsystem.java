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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	/** Creates a new IntakeSubsystem. */
	private static TalonFX intakeMotor, intakeDeployMotor;
	private static double requestedIntakeDeploy;
	public static boolean autonPulse = false, intaking = false, reversing = false;
	static VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withEnableFOC(false);
	private static Timer timer = new Timer();

	public static Command intakePulse = new IntakePulseCommand(false);

	public IntakeSubsystem() {
		intakeMotor = new TalonFX(IDConstants.IntakeMotorID, "CANivore");
		intakeDeployMotor = new TalonFX(IDConstants.IntakeDeployMotorID, "CANivore");
		timer.start();

		intakeMotor.getConfigurator()
				.apply(new CurrentLimitsConfigs()
						.withStatorCurrentLimitEnable(false)
						.withSupplyCurrentLimitEnable(true)
						.withSupplyCurrentLimit(70)
						.withSupplyCurrentLowerLimit(45)
						.withSupplyCurrentLowerTime(2));
		intakeMotor.getConfigurator().apply(
				new TalonFXConfiguration().MotorOutput
						.withInverted(InvertedValue.CounterClockwise_Positive));
		intakeDeployMotor.getConfigurator().apply(
				new TalonFXConfiguration().MotorOutput
						.withInverted(IDConstants.PracticeBot ? InvertedValue.CounterClockwise_Positive: InvertedValue.Clockwise_Positive));

		intakeDeployMotor.setNeutralMode(NeutralModeValue.Brake);
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

		intakeMotor.getConfigurator().apply(new MotionMagicConfigs()
				.withMotionMagicAcceleration(IntakeConstants.IntakeAcceleration));
		intakeMotor.getConfigurator().apply(new SlotConfigs()
				.withKP(IntakeConstants.IntakeP).withKV(IntakeConstants.IntakeV));
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

	public static void setIntakeVoltage(double voltage) {
		intakeMotor.setControl(new VoltageOut(voltage).withEnableFOC(false));
	}

	public static void setIntakeSpeed(double speed) {
		intakeMotor.setControl(velocityVoltageControl.withVelocity(speed));

	}

	public static double getTotalCurrentDraw() {
		double totalDraw = 0;
		totalDraw += intakeMotor.getSupplyCurrent().getValueAsDouble();
		totalDraw += intakeDeployMotor.getSupplyCurrent().getValueAsDouble();
		return totalDraw;
	}

	public static void log() {
		// Slider Position
		SmartDashboard.putNumber("/Intake/Position", getIntakeDeployRotations());
		SmartDashboard.putNumber("/Intake/IntakeTemp", intakeMotor.getDeviceTemp().getValueAsDouble());
		SmartDashboard.putNumber("/Intake/IntakeDeployTemp", intakeDeployMotor.getDeviceTemp().getValueAsDouble());

	}

	@Override
	public void periodic() {
		log();
		if (DriverStation.isTest()) {
			intakeDeployMotor.setControl(new VoltageOut(0));
			setIntakeVoltage(0);
			return;
		}

		double intakePosition = requestedIntakeDeploy;
		boolean shouldPulse = (RobotContainer.operatorController.rightTrigger().getAsBoolean()
				&& !RobotContainer.operatorLeftStickUp.getAsBoolean()
				&& RobotContainer.driveSubsystem.getPose2d().getX() < 5.3
				&& requestedIntakeDeploy > 1)
				|| autonPulse;

		if (timer.get() < 1) {
			// Out
			if (shouldPulse)
				intakePosition = IntakeConstants.IntakeOutPosition;
		}

		else if (timer.get() < 1.5) {
			// In
			if (shouldPulse)
				intakePosition = IntakeConstants.IntakeOutPosition * 0.5;

		} else
			timer.restart();

		intakeDeployMotor.setControl(new MotionMagicVoltage(intakePosition));

		if (intaking || shouldPulse)
			setIntakeVoltage(5);
		else if (reversing)
			setIntakeVoltage(-6);
		else
			setIntakeVoltage(0);
	}
}
