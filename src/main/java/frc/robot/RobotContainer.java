/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.ExampleCommand;
import frc.robot.commands.tankDrive;
import frc.robot.commands.visionTarget;
import frc.robot.commands.vLED;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.limeLight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsytems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final static DriveBase driveBase = new DriveBase();
  private final limeLight visionSensor = new limeLight();

  //Commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final static tankDrive mainDrive = new tankDrive(driveBase);

  //OI Devices
  public static Joystick driver = new Joystick(0);
  public static JoystickButton driver1 = new JoystickButton(driver, 1);
  public static JoystickButton driver2 = new JoystickButton(driver, 2);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    driver1.whenPressed(new vLED(visionSensor, true), false);
    driver1.whenReleased(new vLED(visionSensor, false), false);
    driver2.whileHeld(new visionTarget(visionSensor, driveBase), false);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  } 

  public static void startTankDrive()
  {
    mainDrive.schedule(true);
  }

  public void turnLEDOff()
  {
    visionSensor.vLEDoff();
  }

  public static void initMotor(TalonSRX motor, double peak)
  {
    motor.configPeakOutputForward(peak);
    motor.configPeakOutputReverse(-peak);
    motor.setNeutralMode(NeutralMode.Brake);
  }
  public static void initMotor(VictorSPX motor, double peak)
  {
    motor.configPeakOutputForward(peak);
    motor.configPeakOutputReverse(-peak);
    motor.setNeutralMode(NeutralMode.Brake);
  }
}
