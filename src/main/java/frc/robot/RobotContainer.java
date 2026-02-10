// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Drivetrain.DriveCommand;
import frc.robot.Drivetrain.DriveSubsystem;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Hopper.KickerMotorCommand;
import frc.robot.Hopper.ShootCommand;
import frc.robot.Hopper.SpinnerMotorCommand;
import frc.robot.Intake.IntakeBallCommand;
import frc.robot.Intake.IntakeMoveCommand;
import frc.robot.Intake.IntakeSubsystem;
import frc.robot.Shooter.AngleHoodCommand;
import frc.robot.Shooter.TurretAutoCommand;
import frc.robot.Shooter.TurretBackCommand;
import frc.robot.Shooter.TurretFrontCommand;
import frc.robot.Shooter.TurretLeftCommand;
import frc.robot.Shooter.TurretRightCommand;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Vision.VisionSubsystem;

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


  public static CommandXboxController driverController = new CommandXboxController(1);
  public static CommandXboxController operatorController = new CommandXboxController(0);
  public static Trigger rightDriverTrigger = driverController.rightTrigger(0.5);
  public static Trigger rightDriverBumper = driverController.rightBumper();

  public static SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    putAutons();
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
    operatorController.rightTrigger(0.15).whileTrue(new ShootCommand());

    //Intake Commands 
    operatorController.leftTrigger(.1).onTrue(new IntakeMoveCommand(true));
    operatorController.leftTrigger(.1).whileTrue(new IntakeBallCommand());
    operatorController.leftBumper().onTrue(new IntakeMoveCommand(false));

    driveSubsystem.setDefaultCommand(new DriveCommand());

    // Turret Set Positions
    operatorController.povLeft().onTrue(new TurretLeftCommand());
    operatorController.povRight().onTrue(new TurretRightCommand());
    operatorController.povUp().onTrue(new TurretFrontCommand());
    operatorController.povDown().onTrue(new TurretBackCommand());
    operatorController.a().onTrue(new TurretAutoCommand());
    // climber here ->


  }

  public void putAutons() {
    // NamedCommands.registerCommand("RevWheelCommand", new RevWheelCommand());
    NamedCommands.registerCommand("AngleHoodCommand", new AngleHoodCommand(0));
    NamedCommands.registerCommand("RotateTurretCommand", new TurretAutoCommand());
    NamedCommands.registerCommand("IntakeDeployCommand", new IntakeMoveCommand(true));
    NamedCommands.registerCommand("IntakeRetractCommand", new IntakeMoveCommand(false));
    NamedCommands.registerCommand("IntakeBallCommand", new IntakeBallCommand());
    NamedCommands.registerCommand("SpinnerMotorCommand", new SpinnerMotorCommand());
    NamedCommands.registerCommand("KickerMotorCommand", new KickerMotorCommand());
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
