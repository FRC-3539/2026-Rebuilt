// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicExpoTorqueCurrentFOC_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.EnumConstants.ShootingMode;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX rotateMotor, shootMotorA, shootMotorB;
  private static TalonFXS hoodMotor;
  private static double requestedTurretAngle, requestedHoodAngle;
  public static ShootingMode mode;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    requestedHoodAngle = 0;
    requestedTurretAngle = 0;

    shootMotorA = new TalonFX(IDConstants.ShootMotorAID, "rio");
    shootMotorA.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    shootMotorB = new TalonFX(IDConstants.ShootMotorBID, "rio");
    shootMotorB.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    rotateMotor = new TalonFX(IDConstants.TurretRotateMotorID, "rio");
    rotateMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    hoodMotor = new TalonFXS(IDConstants.HoodMotorID, "rio");
        hoodMotor.getConfigurator()
                .apply(new TalonFXSConfiguration().MotorOutput
                        .withInverted(InvertedValue.Clockwise_Positive));
        hoodMotor.getConfigurator().apply(new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.Minion_JST)));

    rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    rotateMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(degreesToRotations(135.0))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(degreesToRotations(-135)));

    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    hoodMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(degreesToRotations(135.0))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(degreesToRotations(-135)));

    shootMotorB.setControl(new Follower(shootMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
  }


  public static double rotationsToDegrees(double rotations) {
        return rotations * 360 / 47.0;
    }

    public static double degreesToRotations(double degrees) {
        return degrees * 47 / 360.0;
    }

    public static void setRotateAngle(double angle) {
        requestedTurretAngle = angle;
    }

    public static void setHoodAngle(double angle) {
        requestedHoodAngle = angle;
    }

     public static void setShootVelocity(double angularVelocity) {
        shootMotorA.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    }

    public static double getTurretAngle() {
        return rotationsToDegrees(rotateMotor.getPosition().getValueAsDouble());
    }
     public static void reloadConstants() {
        shootMotorA.getConfigurator().apply(new SlotConfigs()
                .withKP(ShooterConstants.ShooterP)
                .withKI(ShooterConstants.ShooterI)
                .withKD(ShooterConstants.ShooterD)
                .withKV(ShooterConstants.ShooterV));
        shootMotorA.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ShooterConstants.ShooterAcceleration)
                .withMotionMagicCruiseVelocity(ShooterConstants.ShooterCruiseVelocity));
        rotateMotor.getConfigurator().apply(new SlotConfigs()
                .withKP(ShooterConstants.TurretRotateP)
                .withKI(ShooterConstants.TurretRotateI)
                .withKD(ShooterConstants.TurretRotateD)
                .withKV(ShooterConstants.TurretRotateV));
        rotateMotor.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ShooterConstants.TurretRotateAcceleration)
                .withMotionMagicCruiseVelocity(ShooterConstants.TurretRotateCruiseVelocity));
        hoodMotor.getConfigurator().apply(new SlotConfigs()
                .withKP(ShooterConstants.HoodP)
                .withKI(ShooterConstants.HoodI)
                .withKD(ShooterConstants.HoodD)
                .withKV(ShooterConstants.HoodV));
        hoodMotor.getConfigurator().apply(new MotionMagicConfigs()
                .withMotionMagicAcceleration(ShooterConstants.HoodAcceleration)
                .withMotionMagicCruiseVelocity(ShooterConstants.HoodCruiseVelocity));
    }


    





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
