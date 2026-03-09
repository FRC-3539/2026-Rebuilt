// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import org.frcteam3539.Byte_Swerve_Lib.math.MathUtils;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.Utilities;
import frc.robot.RobotContainer;
import frc.robot.Constants.EnumConstants.ShootingMode;
import frc.robot.Drivetrain.DriveSubsystem;

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

    private static final double maxShootingSpeed = 39, minHoodAngle = 51, maxHoodAngle = 77, turretXOffset = -0.0697992,
            turretYOffset = -0.1528572, maxTurretRotation = 270, minTurretRotation = -270;

    static StatusSignal<AngularVelocity> velocitySignal;
    static VelocityVoltage velocityVoltageControl = new VelocityVoltage(0).withEnableFOC(true);
    static VoltageOut voltageOutControlTop = new VoltageOut(0).withEnableFOC(true);

    private static double robotVelocityX = 0, robotVelocityY = 0;
    public static boolean revState = false;
    public static boolean hideHoodState = false;
    public static boolean teleopHideHoodState = false;

    /** Creates a new ShooterSubsystem. */
    public ShooterSubsystem() {
        rotateMotor = new TalonFXS(IDConstants.TurretRotateMotorID, "rio");
        shootMotorLeader = new TalonFX(IDConstants.ShootMotorLeaderID, "rio");
        shootMotorFollower = new TalonFX(IDConstants.ShootMotorFollowerID, "rio");
        hoodMotor = new TalonFXS(IDConstants.HoodMotorID, "rio");

        rotateMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30).withSupplyCurrentLowerLimit(20).withSupplyCurrentLowerTime(2));

        shootMotorLeader.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorFollower.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(40).withSupplyCurrentLowerLimit(30).withSupplyCurrentLowerTime(2));

        shootMotorLeader.getConfigurator()
                .apply(new TalonFXConfiguration().MotorOutput
                        .withInverted(InvertedValue.CounterClockwise_Positive));
        shootMotorFollower.getConfigurator()
                .apply(new TalonFXConfiguration().MotorOutput
                        .withInverted(InvertedValue.Clockwise_Positive));

        rotateMotor.getConfigurator()
                .apply(new TalonFXSConfiguration().withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.Minion_JST))
                        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)));

        hoodMotor.getConfigurator().apply(new TalonFXSConfiguration()
                .withCommutation(new CommutationConfigs()
                        .withMotorArrangement(MotorArrangementValue.Minion_JST))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));

        hoodMotor.getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimitEnable(false).withSupplyCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(30).withSupplyCurrentLowerLimit(20).withSupplyCurrentLowerTime(2));

        rotateMotor.setNeutralMode(NeutralModeValue.Brake);
        rotateMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ShooterConstants.TurretRotateSoftMax)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ShooterConstants.TurretRotateSoftMin));

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.getConfigurator()
                .apply(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(ShooterConstants.HoodSoftMax)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(ShooterConstants.HoodSoftMin));

        shootMotorFollower.setControl(new Follower(shootMotorLeader.getDeviceID(), MotorAlignmentValue.Opposed));
        velocitySignal = shootMotorLeader.getVelocity();

        reloadConstants();
    }

    public static double turretRotationsToAngle(double rotations) {
        return rotations * ShooterConstants.TurretDegreesPerRotation;
    }

    public static void zeroHoodMotor() {
        hoodMotor.setPosition(0);
    }

    public static void zeroRotateMotor() {
        rotateMotor.setPosition(0);
    }

    public static boolean turretReady() {
        return MathUtil.isNear(requestedState.rotation, getTurretRotation(), 15)
                && MathUtil.isNear(requestedState.velocity, getShooterVelocity(), 5)
                && MathUtil.isNear(requestedState.hoodAngle, getHoodAngle(), .2);
        // TODO: Make threshhold smaller if it doesnt work properly
    }

    public static double turretAngleToRotations(double degrees) {
        return degrees / ShooterConstants.TurretDegreesPerRotation;
    }

    public static double getTurretRotation() {
        return turretRotationsToAngle(rotateMotor.getPosition().getValueAsDouble());
    }

    public static double getHoodAngle() {
        return hoodMotor.getPosition().getValueAsDouble();
    }

    public static double getRequestedShootVelocity() {
        return requestedState.velocity;
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

    public static Translation2d scaleOffset(Translation2d offset, Rotation2d direction, double scale) {
        double dx = direction.getCos();
        double dy = direction.getSin();

        double dot = offset.getX() * dx + offset.getY() * dy;
        return offset.plus(new Translation2d(dx * (scale - 1) * dot, dy * (scale - 1) * dot));
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
        // double distanceToTarget =
        double distanceToTarget = Math.sqrt(Math.pow(
                turret.getDistance(target), 2)
                + Math.pow(2.05, 2));
        double timeToTarget = -0.000178 * Math.pow(distanceToTarget, 2) + 0.044045 * distanceToTarget + 0.576042 + 0.3;
        // double timeToTarget = -0.007 * (distanceToTarget * distanceToTarget) + 0.265
        // * distanceToTarget + 1.35;

        // Step 3: adjust the target point by the distance the robot travels in the
        // above time

        // TEST
        Translation2d offsetAmount = new Translation2d(-robotVelocityX * timeToTarget, -robotVelocityY * timeToTarget);
        // Translation2d scaledOffset = scaleOffset(offsetAmount,
        // turret.minus(target).getAngle(), 1.1);

        Translation2d offsetTarget = new Translation2d(target.getX(), target.getY()).plus(offsetAmount);

        return offsetTarget;
    }

    public static double calculateHoodShotAngle(double distanceToTarget) {
        double angle = -0.053169 * Math.pow(distanceToTarget, 2) + 0.810263 * distanceToTarget - 0.552642;
        return angle;
    }

    public static double calculateHoodFeedAngle(double distanceToTarget) {
        double angle = 0.000233 * Math.pow(distanceToTarget, 3) - 0.023 * Math.pow(distanceToTarget, 2)
                + 0.786667 * distanceToTarget + 49.3;
        return angle;
    }

    public static double calculateShotSpeed(double distanceToTarget) {
        double speed = (4.87426 + 0.3) * distanceToTarget + 28.18329
                + (Math.max(distanceToTarget - 5.715, 0) / 2.54) * 10;
        return speed;
    }

    public static double calculateFeedSpeed(double distanceToTarget) {
        double speed = 0.4333 * Math.pow(distanceToTarget, 2) + 0.1842 * distanceToTarget + 31.09767;
        return speed;
    }

    public static double getShooterVelocity() {
        velocitySignal.refresh();
        return velocitySignal.getValue().magnitude();
    }

    public static double hoodAngleToRotations(double angle) {
        return ShooterConstants.HoodSoftMax
                - (angle - minHoodAngle) / (maxHoodAngle - minHoodAngle) * ShooterConstants.HoodSoftMax;
    }

    public static double hoodRotationsToAngle(double rotations) {
        return rotations * ShooterConstants.HoodDegreesPerRotation + minHoodAngle;

    }

    public static double velocityToRotations(double velocity) {
        return velocity / ShooterConstants.ShooterVelocityPerRotation;
    }

    public static double rotationsToVelocity(double rotations) {
        return rotations * ShooterConstants.ShooterVelocityPerRotation;

    }

    public static Translation2d getTurretLocation() {
        Pose2d robotPose = RobotContainer.driveSubsystem.getPose2d();
        double angleRadians = robotPose.getRotation().getRadians();

        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        double rotatedX = turretXOffset * cos - turretYOffset * sin;
        double rotatedY = turretXOffset * sin + turretYOffset * cos;

        return new Translation2d(robotPose.getX() + rotatedX, robotPose.getY() + rotatedY);
    }

    public static TurretState getScoringState() {
        TurretState state = new TurretState();

        Translation2d turretLocation = getTurretLocation();
        double distanceToTarget = 0;
        // double distanceToTarget = Math.sqrt(Math.pow(
        // turretLocation.getDistance(ShooterConstants.Hub), 2)
        // + Math.pow(2.05, 2));

        double turretAngle = 0;
        switch (mode) {
            case AUTO:
                // System.out.println(ShooterConstants.Hub);
                // System.out.println(getOffsetTarget(turretLocation, ShooterConstants.Hub));
                // System.out.println("\n\n\n");
                Translation2d offsetTarget = getOffsetTarget(turretLocation, ShooterConstants.Hub);
                distanceToTarget = turretLocation.getDistance(offsetTarget);

                turretAngle = turretLocation.minus(offsetTarget).getAngle().getDegrees() - 180;
                double otherAngle = turretAngle < 0 ? turretAngle + 360 : turretAngle - 360;
                turretAngle = Math.abs(turretAngle - getTurretRotation()) <= Math.abs(otherAngle - getTurretRotation())
                        ? turretAngle
                        : otherAngle;
                break;
            case LEFT:
                turretAngle = -85;
                distanceToTarget = 2;
                // do other stuff
                break;
            case RIGHT:
                turretAngle = 85;
                distanceToTarget = 2;
                // do other stuff
                break;
            case FRONT:
                turretAngle = 0;
                distanceToTarget = 2;
                // facing front
                // do other stuff
                break;
            case BACK:
                turretAngle = 0;
                distanceToTarget = 2;
                // facing front
                // do other stuff
                break;
        }
        double turretTargetAngle = (turretAngle - RobotContainer.driveSubsystem.getPose2d().getRotation().getDegrees());

        double otherAngle = turretTargetAngle < 0 ? turretTargetAngle + 360 : turretTargetAngle - 360;
        turretTargetAngle = Math.abs(turretTargetAngle - getTurretRotation()) <= Math
                .abs(otherAngle - getTurretRotation())
                        ? turretTargetAngle
                        : otherAngle;

        if (turretTargetAngle > 280)
            turretTargetAngle -= 360;
        else if (turretTargetAngle < -175)
            turretTargetAngle += 360;

        state.rotation = turretTargetAngle;

        // Calculate hood angle and shot speed
        state.hoodAngle = calculateHoodShotAngle(distanceToTarget);
        if (turretLocation.getY() > 6.7 && Math.abs(turretLocation.getX() - 4.627) < .4)
            teleopHideHoodState = true;
        else if (turretLocation.getY() < 1.45 && Math.abs(turretLocation.getX() - 4.627) < .4)
            teleopHideHoodState = true;

        else if (RobotContainer.operatorController.rightBumper().getAsBoolean()
                || RobotContainer.driverController.rightBumper().getAsBoolean())
            teleopHideHoodState = true;
        else {
            teleopHideHoodState = false;

        }
        state.velocity = calculateShotSpeed(distanceToTarget);

        return state;
    }

    public static TurretState getFeedingState() {
        TurretState state = new TurretState();

        Translation2d turretLocation = getTurretLocation();
        double distanceToTarget = 0;

        double turretAngle = 0;
        switch (mode) {
            case AUTO:
                Translation2d feedTarget;
                if (RobotContainer.driveSubsystem.getPose2d().getY() < 4.05)
                    feedTarget = ShooterConstants.FeedRight;
                else
                    feedTarget = ShooterConstants.FeedLeft;
                Translation2d offsetTarget = getOffsetTarget(turretLocation, feedTarget);
                distanceToTarget = turretLocation.getDistance(offsetTarget);
                turretAngle = turretLocation.minus(offsetTarget)
                        .getAngle().getDegrees() - 180;
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
        double turretTargetAngle = (turretAngle - RobotContainer.driveSubsystem.getPose2d().getRotation().getDegrees())
                % 360;
        if (turretTargetAngle > 280)
            turretTargetAngle -= 360;
        else if (turretTargetAngle < -175)
            turretTargetAngle += 360;

        state.rotation = turretTargetAngle;

        // Calculate hood angle and shot speed
        state.hoodAngle = 2.5;
        if (turretLocation.getY() > 6.7 && Math.abs(turretLocation.getX() - 11.917) < .4)
            teleopHideHoodState = true;
        else if (turretLocation.getY() < 1.45 && Math.abs(turretLocation.getX() - 11.917) < .4)
            teleopHideHoodState = true;
        else if (RobotContainer.operatorController.rightBumper().getAsBoolean()
                || RobotContainer.driverController.rightBumper().getAsBoolean())
            teleopHideHoodState = true;
        else {
            teleopHideHoodState = false;
        }

        state.velocity = calculateFeedSpeed(distanceToTarget);

        return state;
    }

    public static void log() {
        // What we are telling the turret to do
        SmartDashboard.putNumber("/Turret/RequestedShooterVelocity", requestedState.velocity);
        SmartDashboard.putNumber("/Turret/RequestedTurretRotation", requestedState.rotation);
        SmartDashboard.putNumber("/Turret/RequestedHoodAngle", requestedState.hoodAngle);

        // What the turret is actually doing
        SmartDashboard.putNumber("/Turret/ShooterVelocity", getShooterVelocity());
        SmartDashboard.putNumber("/Turret/TurretRotation", getTurretRotation());
        SmartDashboard.putNumber("/Turret/HoodAngle", getHoodAngle());

        SmartDashboard.putBoolean("/Turret/ManualTurretHide",
                RobotContainer.operatorController.rightBumper().getAsBoolean());
        Utilities.publishTranslation2d("/Turret/Location", getTurretLocation());
    }

    @Override
    public void periodic() {
        Translation2d turretLocation = getTurretLocation();

        if (turretLocation.getX() < 5.3)
            requestedState = getScoringState();
        else
            requestedState = getFeedingState();

        // Apply turret calculations
        // System.out.println(velocityToRotations(requestedState.velocity));
        if (!DriverStation.isTest()) {
            rotateMotor.setControl(new MotionMagicVoltage(turretAngleToRotations(requestedState.rotation)));
            if (hideHoodState || teleopHideHoodState)
                hoodMotor.setControl(new MotionMagicVoltage(0));
            else
                hoodMotor.setControl(new MotionMagicVoltage(Math.min(requestedState.hoodAngle, ShooterConstants.HoodSoftMax)));
            if (revState)
                shootMotorLeader.setControl(velocityVoltageControl.withVelocity(requestedState.velocity));
            else
                shootMotorLeader.setControl(new VoltageOut(0));
        } else {
            shootMotorLeader.setControl(new VoltageOut(0));
            hoodMotor.setControl(new VoltageOut(0));
            rotateMotor.setControl(new VoltageOut(0));
        }

        // shootMotorLeader.setControl(velocityVoltageControl.withVelocity(50));
        // System.out.println(hoodAngleToRotations(minHoodAngle + 5));
        // hoodMotor.setControl(new MotionMagicVoltage(hoodAngleToRotations(minHoodAngle
        // + 5)));

        log();
    }
}
