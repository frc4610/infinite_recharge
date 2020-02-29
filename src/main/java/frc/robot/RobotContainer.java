/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climb;
import frc.robot.commands.delay;
import frc.robot.commands.encoderMovement;
import frc.robot.commands.feedUnjam;
import frc.robot.commands.intakeCells;
import frc.robot.commands.intakePivot;
import frc.robot.commands.launchSystem;
import frc.robot.commands.leftencoderMovement;
import frc.robot.commands.navXTurn;
import frc.robot.commands.slowMode;
import frc.robot.commands.tankDrive;
import frc.robot.commands.vLED;
import frc.robot.commands.visionTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDStrip;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.encoder;
import frc.robot.subsystems.limeLight;
import frc.robot.subsystems.navX;
import frc.robot.subsystems.raspberryCameras;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsytems
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static DriveBase driveBase = new DriveBase();
  public final static navX gyro = new navX();
  public final static limeLight visionSensor = new limeLight();
  public final static Launcher launcher = new Launcher();
  public final static Intake intake = new Intake();
  public final static encoder mainEncoders = new encoder();
  public final static LEDStrip lights = new LEDStrip();
  public final static Climber climber = new Climber();
  public final static raspberryCameras raspberries = new raspberryCameras();
  //Commands
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  public final static tankDrive mainDrive = new tankDrive(driveBase);
  public final static climb mainClimb = new climb(climber);
  public final static launchSystem manualLaunch = new launchSystem(launcher, Constants.indexNEOSpeed, Constants.feedNEOSpeed, false);

  private static boolean slow;

  //OI Devices
  public static Joystick driver = new Joystick(0);
  public static JoystickButton driverXButton = new JoystickButton(driver, 3);
  public static JoystickButton driverAButton = new JoystickButton(driver, 1);
  public static JoystickButton driverBButton = new JoystickButton(driver, 2);
  public static JoystickButton driverYButton = new JoystickButton(driver, 4);
  public static JoystickButton driverLeftBumper = new JoystickButton(driver, 5);
  public static JoystickButton driverRightBumper = new JoystickButton(driver, 6);
  public static JoystickButton driverLeftJoyButton = new JoystickButton(driver, 9);

  public static Joystick operator = new Joystick(1);
  public static JoystickButton operatorXButton = new JoystickButton(operator, 3);
  public static JoystickButton operatorAButton = new JoystickButton(operator, 1);
  public static JoystickButton operatorBButton = new JoystickButton(operator, 2);
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
    slow = true;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    // Configure the button bindings
    driverLeftBumper.whenPressed(new vLED(visionSensor, true), false);
    driverLeftBumper.whenReleased(new vLED(visionSensor, false), false);
    ///driverAButton.whenPressed(new encoderMovement(driveBase, mainEncoders, gyro, navX.getYaw(), 60), false);
    driverRightBumper.whenPressed(new visionTarget(visionSensor, driveBase, launcher, gyro, false), false);
    driverLeftJoyButton.whenPressed(new slowMode());
    //driverLeftJoyButton.whenPressed(new slowMode());
    operatorAButton.whenPressed(new feedUnjam(launcher, intake), false);
    operatorYButton.whenPressed(new intakeCells(intake, .8, false), true);
    operatorBButton.whenPressed(new feedUnjam(launcher, intake), false);
    operatorLeftBumper.whenPressed(new intakePivot(intake, Constants.bottomIntakeEncoderPosition, false), true);
    operatorRightBumper.whenPressed(new intakePivot(intake, Constants.middleIntakeEncoderPosition, false), true);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(Robot.goal.getSelected().equals("Launch from current pos"))
    {
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, 58));
      //This Auto Goal Launches 3 Power Cells, and drives forward, off the initiation line
    }
    else if(Robot.goal.getSelected().equals("Launch from current pos, back"))
    {
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, -58));
      //This Auto Goal Launches 3 Power Cells, and drives forward, off the initiation line
    }
    else if(Robot.goal.getSelected().equals("Launch directly facing port, Regrab Trench, Launch")){
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      //new vLED(visionSensor, true),
      new launchSystem(launcher, Constants.indexNEOSpeed, Constants.feedNEOSpeed, true),
      //new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new leftencoderMovement(driveBase, mainEncoders, 144),
      new intakeCells(intake, 1, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 180, 240),
      new intakeCells(intake, 0, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 180, -48),
      new leftencoderMovement(driveBase, mainEncoders, 144),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false));
      /*This Auto Goal is to be chosen when directly in front of the Power Port, facing it, flushly.
      The code will Launch 3 power cells, conduct a left sweeping turn 180, drive forward, grabbing 3 more 
      power cells, conduct one more left sweeping turn, and fire.
      */
    } 
    else if(Robot.goal.getSelected().equals("Launch Directly in front, facing 180 from Trench, Regrab Trench, Launch")){
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false),
      new navXTurn(gyro, driveBase, 180, true),
      new intakeCells(intake, .5, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 180, 72),
      new intakeCells(intake, 0, true),
      new leftencoderMovement(driveBase, mainEncoders, 76),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false));
      /*This Auto Goal is to be chosen when parallel with the trench, facing in the direction of the Power Port.
      The code will auto aim, launch 3 power cells, turn towards the 3 power cells, picking up the 3 power cells, 
      doing a left sweeping turn, and firing.
      */
    }
    else if(Robot.goal.getSelected().equals("Steal, Launch 5 Power Cells")){
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new intakeCells(intake, .5, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, 24),
      new intakeCells(intake, 0, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, -24),
      new navXTurn(gyro, driveBase, -90, true),
      new encoderMovement(driveBase, mainEncoders, gyro, -90, 24),
      new navXTurn(gyro, driveBase, -180, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false));
      /*This code is to be chosen when the robot is positioned directly in front of opponents
      trench  
      */
    }
    else if(Robot.goal.getSelected().equals("Launch, grab Sheild Generator")){
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, -24),
      new navXTurn(gyro, driveBase, -90, true),
      new intakeCells(intake, .5, true),
      new encoderMovement(driveBase, mainEncoders, gyro, -90, 24),
      new intakeCells(intake, 0, true),
      new encoderMovement(driveBase, mainEncoders, gyro, -90, -24),
      new navXTurn(gyro, driveBase, 0, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false));
    }
    else if(Robot.goal.getSelected().equals("Grab Sheild Generator, Launch")){
      return new SequentialCommandGroup(new delay(0),
      new intakePivot(intake, Constants.bottomIntakeEncoderPosition, true),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, -24),
      new navXTurn(gyro, driveBase, -90, true),
      new intakeCells(intake, .5, true),
      new encoderMovement(driveBase, mainEncoders, gyro, -90, 24),
      new intakeCells(intake, 0, true),
      new encoderMovement(driveBase, mainEncoders, gyro, -90, -24),
      new navXTurn(gyro, driveBase, 0, true),
      new vLED(visionSensor, true),
      new visionTarget(visionSensor, driveBase, launcher, gyro, true),
      new vLED(visionSensor, false));
    }
    else
    {
      return new SequentialCommandGroup(new delay(0),
      new encoderMovement(driveBase, mainEncoders, gyro, 0, 48));
    }
  } 

  public static void startTankDrive()
  {
    mainDrive.schedule(true);
  }

  public static void startClimb()
  {
    mainClimb.schedule(false);
  }

  public static void startManualLaunch()
  {
    manualLaunch.schedule(true);
  }
  public static boolean tankOverride()
  {
    return (Math.abs(driver.getRawAxis(1)) > .02)||(Math.abs(driver.getRawAxis(3)) > .02);
  }

  public void turnLEDOff()
  {
    visionSensor.vLEDoff();
  }

  public static void toggleSlow()
  {
    slow = !slow;
  }

  public static boolean isSlow()
  {
    return slow;
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
  public static void initMotor(TalonFX motor, double peak)
  {
    motor.configPeakOutputForward(peak);
    motor.configPeakOutputReverse(-peak);
    motor.setNeutralMode(NeutralMode.Brake);
  }

public double launcher() 
{
  return 0;
}
  public static boolean stateOfFeed(){
  if(launcher.GetIR() <= 15)
   {
      return true;
   }
  else
    {
      return false;
    }
  } 

}
