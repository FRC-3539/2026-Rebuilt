// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Climber.*;
import frc.robot.Constants.AlignConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EnumConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Drivetrain.*;
import frc.robot.Hopper.*;
import frc.robot.Intake.*;
import frc.robot.LED.LEDConstants;
import frc.robot.LED.LEDSubsystem;
import frc.robot.Shooter.*;
import frc.robot.Vision.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static DriveSubsystem driveSubsystem = TunerConstants.createDrivetrain();
  public static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public static VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static HopperSubsystem hopperSubsystem = new HopperSubsystem();
  public static LEDSubsystem ledSubsystem = new LEDSubsystem(true);

  public static AlignConstants alignConstants = new AlignConstants();
  public static ClimberConstants climberConstants = new ClimberConstants();
  public static DriveConstants driveConstants = new DriveConstants();
  public static EnumConstants enumConstants = new EnumConstants();
  public static HopperConstants hopperConstants = new HopperConstants();
  public static IDConstants idConstants = new IDConstants();
  public static IntakeConstants intakeConstants = new IntakeConstants();
  public static LEDConstants ledConstants = new LEDConstants();
  public static ShooterConstants shooterConstants = new ShooterConstants();

  public static CommandXboxController driverController = new CommandXboxController(1);
  public static CommandXboxController operatorController = new CommandXboxController(0);
  public static Trigger leftDriverTrigger = driverController.leftTrigger(0.5);
  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
  public static Trigger rightDriverBumper = driverController.rightBumper();
  public static Trigger operatorLeftStickUp = operatorController.axisLessThan(1, -0.5);
  public static Trigger operatorRightStickUp = operatorController.axisLessThan(1, -0.5);

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putAutons();
    putCommands();
    visionSubsystem.start();
  }

  public void putCommands() {
    SmartDashboard.putData(new ZeroHoodMotorCommand().ignoringDisable(true));
    SmartDashboard.putData(new ZeroIntakeDeployCommand().ignoringDisable(true));
    SmartDashboard.putData(new ZeroTurretRotateCommand().ignoringDisable(true));
    SmartDashboard.putData(new ZeroClimberCommand().ignoringDisable(true));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveSubsystem.setDefaultCommand(new DriveCommand());

    operatorController.rightTrigger(0.15).whileTrue(new ParallelCommandGroup(new ShootCommand(), new RevCommand()));
    operatorController.leftTrigger(0.15).whileTrue(new RevCommand());

    // Intake Commands
    // operatorController.leftTrigger(.15).onTrue(new IntakeMoveCommand(true));
    operatorLeftStickUp.whileTrue(new IntakeBallCommand());
    operatorLeftStickUp.onTrue(new InstantCommand(() -> IntakeSubsystem.intakePulse.cancel()).andThen(new SetIntakeCommand(IntakeConstants.IntakeDeploySoftMax)));
    operatorController.axisGreaterThan(1, 0.5).onTrue(new SetIntakeCommand(IntakeConstants.IntakeDeploySoftMin));

    // Drive Set Positions

    driverController.b().whileTrue(new ParallelDeadlineGroup(
        new PIDAutoAlignCommand(false).andThen(new PIDAutoAlignCommand(true))));

    // driverController.a().whileTrue(new PIDAutoAlignCommand());

    // Turret Set Positions
    operatorController.x().onTrue(new TurretLeftCommand());
    operatorController.b().onTrue(new TurretRightCommand());
    operatorController.y().onTrue(new TurretFrontCommand());
    operatorController.a().onTrue(new TurretAutoCommand());

    operatorController.back().whileTrue(new ReverseHopperCommand());
    operatorController.start().whileTrue(new ReverseIntakeCommand());
    
    // climber
    climberSubsystem.setDefaultCommand(new ClimberCommand());
    operatorController.povUp().onTrue(new SetClimberPositionCommand(ClimberConstants.ClimbHeight));
    operatorController.povRight().onTrue(new SetClimberPositionCommand(ClimberConstants.ClimberAirHeight));
    operatorController.povDown().onTrue(new SetClimberPositionCommand(ClimberConstants.ClimberSoftMin));

  }

  public void putAutons() {
    // NamedCommands.registerCommand("RevWheelCommand", new RevWheelCommand());
    NamedCommands.registerCommand("IntakeDeployCommand", new IntakeMoveCommand(true));
    NamedCommands.registerCommand("IntakeRetractCommand", new IntakeMoveCommand(false));
    NamedCommands.registerCommand("IntakeBallCommand", new IntakeBallCommand());
    NamedCommands.registerCommand("ShootCommand", new ShootCommand());
    NamedCommands.registerCommand("RevCommand", new RevCommand());
    NamedCommands.registerCommand("HideHoodCommand", new HideHoodCommand());
    NamedCommands.registerCommand("AutonPulseCommand", new AutonPulseCommand());
    NamedCommands.registerCommand("ClimberCommand", new ClimberCommand());
    NamedCommands.registerCommand("ClimbDeployCommand", new SetClimberPositionCommand(ClimberConstants.ClimbHeight));
    NamedCommands.registerCommand("ClimbLiftCommand", new SetClimberPositionCommand(ClimberConstants.ClimberAirHeight));
    // NamedCommands.registerCommand("RevWheelCommand", new RevWheelCommand());

    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return chooser.getSelected();
  }
}
