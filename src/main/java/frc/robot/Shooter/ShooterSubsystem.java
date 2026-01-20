// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicExpoTorqueCurrentFOC_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static TalonFX hoodMotor, rotateMotor, shootMotorA, shootMotorB;
  private static double requestedTurretAngle, requestedHoodAngle;
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    requestedHoodAngle = 0;
    requestedTurretAngle = 0;

    shootMotorA = new TalonFX(1, "rio");
    shootMotorA.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    shootMotorB = new TalonFX(2, "rio");
    shootMotorB.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    rotateMotor = new TalonFX(3, "rio");
    rotateMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

    hoodMotor = new TalonFX(4, "rio");
    hoodMotor.getConfigurator()
        .apply(new TalonFXConfiguration().MotorOutput
        .withInverted(InvertedValue.Clockwise_Positive));

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
    //  public static void reloadConstants() {
    //     shootMotor.getConfigurator().apply(new SlotConfigs()
    //             .withKP(ShooterConstants.shootkP)
    //             .withKI(ShooterConstants.shootkI)
    //             .withKD(ShooterConstants.shootkD)
    //             .withKV(ShooterConstants.shootkV));
    //     shootMotor.getConfigurator().apply(new MotionMagicConfigs()
    //             .withMotionMagicAcceleration(ShooterConstants.shootAcceleration));
    //     turretRotateMotor.getConfigurator().apply(new SlotConfigs()
    //             .withKP(ShooterConstants.turretRotatekP)
    //             .withKI(ShooterConstants.turretRotatekI)
    //             .withKD(ShooterConstants.turretRotatekD)
    //             .withKV(ShooterConstants.turretRotatekV));
    //     turretRotateMotor.getConfigurator().apply(new MotionMagicConfigs()
    //             .withMotionMagicAcceleration(ShooterConstants.turretRotateAcceleration)
    //             .withMotionMagicCruiseVelocity(ShooterConstants.turretRotateCruiseVelocity));
    //     hoodMotor.getConfigurator().apply(new SlotConfigs()
    //             .withKP(ShooterConstants.hoodkP)
    //             .withKI(ShooterConstants.hoodkI)
    //             .withKD(ShooterConstants.hoodkD)
    //             .withKV(ShooterConstants.hoodkV));
    //     hoodMotor.getConfigurator().apply(new MotionMagicConfigs()
    //             .withMotionMagicAcceleration(ShooterConstants.hoodAcceleration)
    //             .withMotionMagicCruiseVelocity(ShooterConstants.hoodCruiseVelocity));
    // }


    





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
