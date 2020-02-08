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
import frc.robot.commands.launchSystem;
import frc.robot.commands.intakeCells;
import frc.robot.commands.intakePivot;
import frc.robot.commands.encoderMovement;
import frc.robot.commands.navXTurn;
import frc.robot.commands.tankDrive;
import frc.robot.commands.visionTarget;
import frc.robot.commands.vLED;

import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.navX;
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
  private static final navX gyro = new navX();
  private final limeLight visionSensor = new limeLight();
  private final Launcher launcher = new Launcher();
  private final static Intake intake = new Intake();
  public static final encoder mainEncoders = new encoder();


  //Commands
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final static tankDrive mainDrive = new tankDrive(driveBase);

  //OI Devices
  public static Joystick driver = new Joystick(0);
  public static JoystickButton driverXButton = new JoystickButton(driver, 1);
  public static JoystickButton driverAButton = new JoystickButton(driver, 2);
  public static JoystickButton driverBButton = new JoystickButton(driver, 3);
  public static JoystickButton driverYButton = new JoystickButton(driver, 4);
  public static JoystickButton driverLeftBumper = new JoystickButton(driver, 5);
  public static JoystickButton driverRightBumper = new JoystickButton(driver, 6);
  public static JoystickButton driverLeftTrigger = new JoystickButton(driver, 7);
  public static JoystickButton driverRightTrigger = new JoystickButton(driver, 8);
  public static JoystickButton driverBackButton = new JoystickButton(driver, 9);

  public static Joystick operator = new Joystick(1);
  public static JoystickButton operatorXButton = new JoystickButton(operator, 1);
  public static JoystickButton operatorAButton = new JoystickButton(operator, 2);
  public static JoystickButton operatorBButton = new JoystickButton(operator, 3);
  public static JoystickButton operatorYButton = new JoystickButton(operator, 4);
  public static JoystickButton operatorLeftBumper = new JoystickButton(operator, 5);
  public static JoystickButton operatorRightBumper = new JoystickButton(operator, 6);
  public static JoystickButton operatorLeftTrigger = new JoystickButton(operator, 7);
  public static JoystickButton operatorRightTrigger = new JoystickButton(operator, 8);
  public static JoystickButton operatorBackButton = new JoystickButton(operator, 9);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    driverLeftBumper.whenPressed(new vLED(visionSensor, true), false);
    driverLeftBumper.whenReleased(new vLED(visionSensor, false), false);
    driverRightBumper.whenPressed(new visionTarget(visionSensor, driveBase, launcher, true), false);
    driverXButton.whenPressed(new navXTurn(gyro, driveBase, -90), true);
    driverBButton.whenPressed(new navXTurn(gyro, driveBase, 90), true);
    driverYButton.whenPressed(new navXTurn(gyro, driveBase, 180), true);
    driverAButton.whenPressed(new encoderMovement(driveBase, mainEncoders, gyro, 60), false);
    driverLeftTrigger.whileHeld(new visionTarget(visionSensor, driveBase, launcher, false), false);
    driverRightTrigger.whileHeld(new launchSystem(launcher, Constants.indexNEOSpeed , Constants.feedNEOSpeed, Constants.launchNEOSpeed) , true);
    operatorYButton.whenPressed(new intakeCells(intake, .5), true);
    operatorLeftBumper.whenPressed(new intakePivot(intake, Constants.bottomIntakeEncoderPosition), true);
    operatorRightBumper.whenPressed(new intakePivot(intake, Constants.middleIntakeEncoderPosition), true);
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

  public static boolean tankOverride()
  {
    return (Math.abs(driver.getRawAxis(1)) > .02)||(Math.abs(driver.getRawAxis(3)) > .02);
  }

  /**
   * Determines if the robot is blocked/being pushed
   * @return If the robot is blocked
   */
  public static boolean robotBlocked()
  {
    return !(mainEncoders.drivebaseIsMoving() == gyro.getMoving());
  }

  public void turnLEDOff()
  {
    visionSensor.vLEDoff();
  }

  public static double pivotEncoder()
  {
    return intake.getPivotEncoderVaule();
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
