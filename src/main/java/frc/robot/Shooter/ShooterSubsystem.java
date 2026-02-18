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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.Utilities;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnumConstants.ShootingMode;

class TurretState {
    public double rotation;
    public double hoodAngle;
    public double velocity;
}

public class ShooterSubsystem extends SubsystemBase {
    private static TalonFX shootMotorLeader, shootMotorFollower;
    private static TalonFXS hoodMotor, rotateMotor;
    public static ShootingMode mode = ShootingMode.AUTO;

    public static TurretState requestedState;

    private static final double maxShootingSpeed = 39, minHoodAngle = 51, maxHoodAngle = 77, turretXOffset = 0.1528572,
            turretYOffset = -0.0697992, maxTurretRotation = 270, minTurretRotation = -270;

    static StatusSignal<AngularVelocity> velocitySignal;
    static VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withEnableFOC(true);
    static VoltageOut voltageOutControlTop = new VoltageOut(0).withEnableFOC(true);

    private static double robotVelocityX = 0, robotVelocityY = 0;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        rotateMotor = new TalonFXS(IDConstants.TurretRotateMotorID, "rio");
        shootMotorLeader = new TalonFX(IDConstants.ShootMotorLeaderID, "rio");
        shootMotorFollower = new TalonFX(IDConstants.ShootMotorFollowerID, "rio");
        hoodMotor = new TalonFXS(IDConstants.HoodMotorID, "rio");
        
        rotateMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorLeader.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorFollower.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(60).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorLeader.getConfigurator()
                .apply(new TalonFXConfiguration().MotorOutput
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        shootMotorFollower.getConfigurator()
                .apply(new TalonFXConfiguration().MotorOutput
                        .withInverted(InvertedValue.Clockwise_Positive));

        
        rotateMotor.getConfigurator()
                .apply(new TalonFXConfiguration().MotorOutput
                        .withInverted(InvertedValue.Clockwise_Positive));
        hoodMotor.getConfigurator()
                .apply(new TalonFXSConfiguration().MotorOutput
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        hoodMotor.getConfigurator().apply(new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.Minion_JST)));

        rotateMotor.setNeutralMode(NeutralModeValue.Brake);
        rotateMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(turretAngleToRotations(maxTurretRotation))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(turretAngleToRotations(minTurretRotation)));
                        
        rotateMotor.getConfigurator().apply(new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.Minion_JST)));

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(turretAngleToRotations(ShooterConstants.HoodSoftMax))
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(turretAngleToRotations(ShooterConstants.HoodSoftMin)));

        shootMotorFollower.setControl(new Follower(shootMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        velocitySignal = shootMotorLeader.getVelocity();

        reloadConstants();
    }

   
    public static double turretRotationsToAngle(double rotations) {
        return rotations * ShooterConstants.TurretDegreesPerRotation;
    }

    
    public static double turretAngleToRotations(double degrees) {
        return degrees / ShooterConstants.TurretDegreesPerRotation;
    }

    public static double getTurretRotation() {
        return turretRotationsToAngle(rotateMotor.getPosition().getValueAsDouble());
    }
     public static double getHoodAngle() {
        return hoodRotationsToAngle(hoodMotor.getPosition().getValueAsDouble());
    }

    public static void reloadConstants() {
        shootMotorLeader.getConfigurator().apply(new SlotConfigs()
                .withKP(ShooterConstants.ShooterP)
                .withKI(ShooterConstants.ShooterI)
                .withKD(ShooterConstants.ShooterD)
                .withKV(ShooterConstants.ShooterV));
        shootMotorLeader.getConfigurator().apply(new MotionMagicConfigs()
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

    public static Translation2d getOffsetTarget(Translation2d turret, Translation2d target) {
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
                turret.getDistance(target), 2)
                + Math.pow(2.05, 2));
        double timeToTarget = -0.000178 * Math.pow(distanceToTarget, 2) + 0.044045 * distanceToTarget + 0.576042;
        // double timeToTarget = -0.007 * (distanceToTarget * distanceToTarget) + 0.265
        // * distanceToTarget + 1.35;

        // Step 3: adjust the target point by the distance the robot travels in the
        // above time
        Translation2d offsetTarget = new Translation2d(target.getX() - robotVelocityX * timeToTarget,
                target.getY() - robotVelocityY * timeToTarget);

        return offsetTarget;
    }

    public static double calculateHoodShotAngle(double distanceToTarget) {
        double angle = 0.07 * Math.pow(distanceToTarget, 2) - 2.45 * distanceToTarget + 82;
        return angle;
    }

    public static double calculateHoodFeedAngle(double distanceToTarget) {
        double angle = 0.000233 * Math.pow(distanceToTarget, 3) - 0.023 * Math.pow(distanceToTarget, 2) + 0.786667 * distanceToTarget + 49.3;
        return angle;
    }

    public static double calculateShotSpeed(double distanceToTarget) {
        double speed = 0.65 * distanceToTarget + 16.33;
        return speed;
    }

    public static double calculateFeedSpeed(double distanceToTarget) {
        double speed = -0.008 * Math.pow(distanceToTarget, 2) + 1.066 * distanceToTarget + 7.8;
        return speed;
    }

    public static double getShooterVelocity() {
        velocitySignal.refresh();
        return velocitySignal.getValue().magnitude();
    }

    public static double hoodAngleToRotations(double angle) {
        return (angle - minHoodAngle) / ShooterConstants.HoodDegreesPerRotation;
    }

    public static double hoodRotationsToAngle(double rotations) {
        return  rotations * ShooterConstants.HoodDegreesPerRotation + minHoodAngle;

    }

    public static double velocityToRotations(double velocity) {
        return velocity / ShooterConstants.ShooterVelocityPerRotation;
    }

    public static double rotationsToVelocity(double rotations) {
        return rotations * ShooterConstants.ShooterVelocityPerRotation ;

    }

    public static Translation2d getTurretLocation() {
        Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();
        double angleRadians = Math.toRadians(robotPose.getRotation().getRadians());

        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        double rotatedX = turretXOffset * cos - turretYOffset * sin;
        double rotatedY = turretXOffset * sin + turretYOffset * cos;

        return new Translation2d(robotPose.getX() + rotatedX, robotPose.getY() + rotatedY);
    }

    public static TurretState getScoringState() {
        TurretState state = new TurretState();

        Translation2d turretLocation = getTurretLocation();
        double distanceToTarget = Math.sqrt(Math.pow(
                turretLocation.getDistance(ShooterConstants.Hub), 2)
                + Math.pow(2.05, 2));

        double turretAngle = 0;
        switch (mode) {
            case AUTO:
                turretAngle = turretLocation.minus(getOffsetTarget(turretLocation, ShooterConstants.Hub)).getAngle().getDegrees() - 180;
                double otherAngle = turretAngle < 0 ? turretAngle + 360 : turretAngle - 360;
                turretAngle = Math.abs(turretAngle - getTurretRotation()) <= Math.abs(otherAngle - getTurretRotation())
                        ? turretAngle
                        : otherAngle;
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
        double turretTargetAngle = (RobotContainer.driveSubsystem.getPose2d().getRotation().getDegrees() - turretAngle)
                % 360;
        if (turretTargetAngle > 260)
            turretTargetAngle -= 360;
        else if (turretTargetAngle < -260)
            turretTargetAngle += 360;

        state.rotation = turretTargetAngle;

        // Calculate hood angle and shot speed
        state.hoodAngle = calculateHoodShotAngle(distanceToTarget);
        if (turretLocation.getY() > 6.7 && Math.abs(turretLocation.getX() - 4.627) < .4)
            state.hoodAngle = minHoodAngle;
        if (turretLocation.getY() < 1.45 && Math.abs(turretLocation.getX() - 4.627) < .4)
            state.hoodAngle = minHoodAngle;
        if (RobotContainer.operatorController.rightBumper().getAsBoolean())
            state.hoodAngle = minHoodAngle;

        state.velocity = calculateShotSpeed(distanceToTarget);

        return state;
    }

    public static TurretState getFeedingState() {
        TurretState state = new TurretState();

        Translation2d turretLocation = getTurretLocation();
        double distanceToTarget = Math.sqrt(Math.pow(
                turretLocation.getDistance(ShooterConstants.FeedMiddle), 2)
                + Math.pow(2.05, 2));

        double turretAngle = 0;
        switch (mode) {
            case AUTO:
                turretAngle = turretLocation.minus(getOffsetTarget(turretLocation, ShooterConstants.FeedMiddle)).getAngle().getDegrees() - 180;
                double otherAngle = turretAngle < 0 ? turretAngle + 360 : turretAngle - 360;
                turretAngle = Math.abs(turretAngle - getTurretRotation()) <= Math.abs(otherAngle - getTurretRotation())
                        ? turretAngle
                        : otherAngle;
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
        double turretTargetAngle = (RobotContainer.driveSubsystem.getPose2d().getRotation().getDegrees() - turretAngle)
                % 360;
        if (turretTargetAngle > 260)
            turretTargetAngle -= 360;
        else if (turretTargetAngle < -260)
            turretTargetAngle += 360;

        state.rotation = turretTargetAngle;

        // Calculate hood angle and shot speed
        state.hoodAngle = calculateHoodFeedAngle(distanceToTarget);
        if (turretLocation.getY() > 6.7 && Math.abs(turretLocation.getX() - 11.917) < .4)
            state.hoodAngle = minHoodAngle;
        if (turretLocation.getY() < 1.45 && Math.abs(turretLocation.getX() - 11.917) < .4)
            state.hoodAngle = minHoodAngle;
        if (RobotContainer.operatorController.rightBumper().getAsBoolean())
            state.hoodAngle = minHoodAngle;

        state.velocity = calculateFeedSpeed(distanceToTarget);

        return state;
    }
    
    public static void log(){
        // What we are telling the turret to do
        SmartDashboard.putNumber("/Turret/RequestedShooterVelocity", requestedState.velocity);
        SmartDashboard.putNumber("/Turret/RequestedTurretRotation", requestedState.rotation);
        SmartDashboard.putNumber("/Turret/RequestedHoodAngle", requestedState.hoodAngle);
        
        // What the turret is actually doing 
        SmartDashboard.putNumber("/Turret/ShooterVelocity", getShooterVelocity());
        SmartDashboard.putNumber("/Turret/TurretRotation", getTurretRotation());
        SmartDashboard.putNumber("/Turret/HoodAngle", getHoodAngle());

        SmartDashboard.putBoolean("/Turret/ManualTurretHide", RobotContainer.operatorController.rightBumper().getAsBoolean());
        Utilities.publishTranslation2d("/Turret/Location",getTurretLocation());
    }

    @Override
    public void periodic() {
        Translation2d turretLocation = getTurretLocation();

        if (turretLocation.getX() < 5.3) requestedState = getScoringState();
        else requestedState = getFeedingState();

        // Apply turret calculations
        // rotateMotor.setControl(new MotionMagicVoltage(turretAngleToRotations(requestedState.rotation)));
        // hoodMotor.setControl(new MotionMagicVoltage(hoodAngleToRotations(requestedState.hoodAngle)));
        //  shootMotorLeader.setControl(velocityVoltageControl.withVelocity(velocityToRotations(requestedState.velocity)));



        //  shootMotorLeader.setControl(velocityVoltageControl.withVelocity(20));
        // System.out.println(hoodAngleToRotations(minHoodAngle + 5));
                hoodMotor.setControl(new MotionMagicVoltage(hoodAngleToRotations(minHoodAngle + 5)));


        log();
    }
}
