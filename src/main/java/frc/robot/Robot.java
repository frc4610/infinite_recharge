/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpiutil.net.PortForwarder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;
  public static SendableChooser<String> goal = new SendableChooser<>();
  private double Straighten;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    RobotContainer.climber.limitSetup(false);
    PortForwarder.add(5801, "limelight.local", 5801);
    SmartDashboard.putNumber("Delay", 0);
    SmartDashboard.putNumber("Manual Launch Power", .5);
    goal.addOption("Drive Forward", "df");
    goal.setDefaultOption("Drive Forward", "df");
    goal.addOption("Launch from current pos", "Launch from current pos");
    goal.addOption("Launch from current pos, back", "Launch from current pos, back");
    //goal.addOption("Launch Directly in front, facing 180 from Trench, Regrab Trench, Launch", "Launch Directly in front, facing 180 from Trench, Regrab Trench, Launch");
    goal.addOption("Launch directly facing port, Regrab Trench, Launch", "Launch directly facing port, Regrab Trench, Launch");
    goal.addOption("Steal, Launch 5 Power Cells", "Steal, Launch 5 Power Cells");
    //goal.addOption("Launch, grab Sheild Generator", "Launch, grab Sheild Generator");
    //goal.addOption("Grab Sheild Generator, Launch", "Grab Sheild Generator, Launch");
    SmartDashboard.putData("Auto Goal", goal);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    CameraServer.getInstance().startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("Limit", RobotContainer.climber.limitState());
    //RobotContainer.lights.setLEDPulse(0, 5);
    RobotContainer.lights.setLEDRainbow();
    SmartDashboard.putData("Auto Goal", goal);
    SmartDashboard.putBoolean("Is Slow", RobotContainer.isSlow());
    SmartDashboard.putNumber("Climb Position", RobotContainer.climber.getEnc());
    SmartDashboard.putNumber("Pivot", RobotContainer.intake.getPivotEncoderVaule());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  @Override
  public void disabledPeriodic() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    RobotContainer.startClimb();
    RobotContainer.isAuto(true);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    RobotContainer.gyro.resetGyro();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    RobotContainer.isAuto(false);
    RobotContainer.startClimb();
    RobotContainer.startClimbTimer();
    RobotContainer.intake.neutralMotors();
    RobotContainer.driveBase.setOpenLoopRamp(.75);
    RobotContainer.startTankDrive();
    RobotContainer.startManualLaunch();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Gyro", Straighten);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
