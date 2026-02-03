// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicExpoTorqueCurrentFOC_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EnumConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnumConstants.ShootingMode;

public class ShooterSubsystem extends SubsystemBase {
    private static TalonFX rotateMotor, shootMotorA, shootMotorB;
    private static TalonFXS hoodMotor;
    private static double requestedTurretAngle, requestedHoodAngle;
    public static ShootingMode mode;

    private static final double maxShootingSpeed = 39, minHoodAngle = 51, maxHoodAngle = 77;
    
	static StatusSignal<AngularVelocity> velocitySignal;
	static VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withEnableFOC(true);
	static VoltageOut voltageOutControlTop = new VoltageOut(0).withEnableFOC(true);

    private static double robotVelocityX = 0, robotVelocityY = 0;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {

        rotateMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorA.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorB.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

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
                        .withForwardSoftLimitThreshold(turretAngleToRotations(135.0))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(turretAngleToRotations(-135)));

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(turretAngleToRotations(135.0))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(turretAngleToRotations(-135)));

        shootMotorB.setControl(new Follower(shootMotorA.getDeviceID(), MotorAlignmentValue.Aligned));
		velocitySignal = shootMotorA.getVelocity();
    }

    public static double turretRotationsToAngle(double rotations) {
        return rotations * 360 / 47.0;
    }

    public static double turretAngleToRotations(double degrees) {
        return degrees * 47 / 360.0;
    }

    public static void setRotateAngle(double angle) {
        requestedTurretAngle = angle;
    }

    public static void setHoodAngle(double angle) {
        requestedHoodAngle = angle;
    }

    // public static void setShootVelocity(double angularVelocity) {
    //     shootMotorA.setControl(new MotionMagicVelocityVoltage(angularVelocity));
    // }

    public static double getTurretAngle() {
        return turretRotationsToAngle(rotateMotor.getPosition().getValueAsDouble());
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

    public static Translation2d getOffsetTarget() {
        // Step 1: calculate robot velocity
        robotVelocityX = ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.driveSubsystem.getKinematics()
                        .toChassisSpeeds(RobotContainer.driveSubsystem.getState().ModuleStates),
                RobotContainer.driveSubsystem.getPose2d().getRotation()).vxMetersPerSecond;
        robotVelocityY = ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.driveSubsystem.getKinematics()
                        .toChassisSpeeds(RobotContainer.driveSubsystem.getState().ModuleStates),
                RobotContainer.driveSubsystem.getPose2d().getRotation()).vyMetersPerSecond;

        // Step 2: Calculate time for shot to hit target from distance
        double distanceToTarget = Math.sqrt(Math.pow(
                RobotContainer.driveSubsystem.getPose2d().getTranslation().getDistance(ShooterConstants.Hub), 2)
                + Math.pow(2.05, 2));
        double timeToTarget = 0.125 * distanceToTarget + 1.9333;
        // double timeToTarget = -0.007 * (distanceToTarget * distanceToTarget) + 0.265
        // * distanceToTarget + 1.35;

        // Step 3: adjust the target point by the distance the robot travels in the
        // above time
        Translation2d offsetTarget = new Translation2d(ShooterConstants.Hub.getX() - robotVelocityX * timeToTarget,
                ShooterConstants.Hub.getY() - robotVelocityY * timeToTarget);

        return offsetTarget;
    }

    public static double calculateHoodAngle(double distanceToTarget){
        double angle = 0.07 * Math.pow(distanceToTarget, 2) - 2.45 * distanceToTarget + 82;
        return angle;
    }

    public static double calculateShotSpeed(double distanceToTarget){
        double speed = 0.65 * distanceToTarget + 16.33;
       return speed;
    }

    public static double getShooterSpeed() {
		velocitySignal.refresh();
		return velocitySignal.getValue().magnitude();
	}

    // TODO
    public static double hoodAngleToRotations(double angle) {
        return 0;
    }
    // TODO
    public static double hoodRotationsToAngle(double rotations) {
        return 0;

    }
    //TBD
    public static double velocityToRotations(double angle) {
        return 0;
    }
    //TBD
    public static double rotationsToVelocity(double rotations) {
        return 0;

    }

    @Override
    public void periodic() {

        // Calculate turret angle
        double turretAngle = 0;
        switch (mode) {
            case AUTO:
                turretAngle = RobotContainer.driveSubsystem.getPose2d().getTranslation()
                    .minus(getOffsetTarget()).getAngle().getDegrees() - 180;
                double otherAngle = turretAngle < 0 ? turretAngle + 360 : turretAngle - 360;
                turretAngle = Math.abs(turretAngle - getTurretAngle()) <= Math.abs(otherAngle - getTurretAngle()) 
                    ? turretAngle : otherAngle;
                break;
            case LEFT:
                turretAngle = -85;
                // do other stuff
                break;
            case RIGHT:
                turretAngle = 85;
                // do other stuff
                break;
            case FRONT:
                turretAngle = 0;
                // facing front
                // do other stuff
                break;
            case BACK:
                turretAngle = 0;
                // facing front
                // do other stuff
                break;
        }
        double turretTargetAngle = (RobotContainer.driveSubsystem.getPose2d().getRotation().getDegrees() - turretAngle) % 360;
        if (turretTargetAngle > 260) turretTargetAngle -= 360;
        else if (turretTargetAngle < -260) turretTargetAngle += 360;

        // Calculate hood angle and shot speed
        double distanceToTarget = Math.sqrt(Math.pow(
            RobotContainer.driveSubsystem.getPose2d().getTranslation().getDistance(ShooterConstants.Hub), 2)
            + Math.pow(2.05, 2));
        double hoodAngle = calculateHoodAngle(distanceToTarget);
        double shotSpeed = calculateShotSpeed(distanceToTarget);

        // Apply turret calculations
        rotateMotor.setControl(new MotionMagicVoltage(turretAngleToRotations(turretTargetAngle)));
        hoodMotor.setControl(new MotionMagicVoltage(hoodAngleToRotations(hoodAngle)));
		shootMotorA.setControl(velocityVoltageControl.withVelocity(velocityToRotations(shotSpeed)));
    }
}
