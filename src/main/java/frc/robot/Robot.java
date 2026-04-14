// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private AnalogInput tempSensor;

  // Constants for NRSE104F3950B1F
  private final double B_COEFFICIENT = 3950.0; // B25/50 from datasheet
  private final double R_NOMINAL = 10000.0; // 10k ohms
  private final double T_NOMINAL = 298.15; // 25°C in Kelvin
  private final double R_FIXED = 10000.0; // Your 10k divider resistor

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    RobotController.setBrownoutVoltage(6.75);
    tempSensor = new AnalogInput(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("/Robot/TotalCurrentDraw", m_robotContainer.getTotalCurrentDraw());
    double voltage = tempSensor.getVoltage();

    // 1. Calculate Resistance
    // Rearranged voltage divider: R_therm = R_fixed * (Vin / Vout - 1)
    double resistance = (5*(10.0/voltage))*1000;

    // 2. Beta Equation calculation
    double steinhart;
    steinhart = Math.log(resistance / R_NOMINAL); // ln(R/Ro)
    steinhart /= B_COEFFICIENT; // 1/B * ln(R/Ro)
    steinhart += 1.0 / T_NOMINAL; // + (1/To)
    steinhart = 1.0 / steinhart; // Invert to get Kelvin

    // 3. Convert to Celsius
    double tempC = steinhart - 273.15;
    SmartDashboard.putNumber("/Robot/TempSensor", resistance);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.intakeSubsystem.setIntakeDeployPosition(RobotContainer.intakeSubsystem.getIntakeDeployRotations());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    RobotContainer.driveSubsystem.setDriveCurrentLimits(70,90);


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.driveSubsystem.setDriveCurrentLimits(30,90);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
