/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.limeLight;

public class visionTarget extends CommandBase {
  private limeLight limeL;
  private DriveBase driveBase;
  private Launcher launcher;
  private Timer timer;

  private double distanceToPowerPort;
  private double xValueOff;
  private double launchSpeed;
  private double maxSpeed;
  private double windSpeed;
  private double leftSpeed;
  private double rightSpeed;

  private boolean launch;
  private boolean positioningMovment;

  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL, DriveBase tdriveBase, Launcher tLauncher, boolean Launch) 
  {
    leftSpeed = 0;
    rightSpeed = 0;
    driveBase = tdriveBase;
    limeL = plimeL;
    launcher = tLauncher;
    timer = new Timer();
    launch = Launch;
    addRequirements(tLauncher);
    addRequirements(tdriveBase);
    addRequirements(plimeL);
    launchSpeed = 0;
    windSpeed = Constants.windSpeedNEO;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    positioningMovment = false;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = limeL.getXValueOff();
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);

    //Aims the robot at 
    if(Math.abs(xValueOff) > 2)
      {
        leftSpeed = Constants.kp*xValueOff;// - Constants.minPower;
        rightSpeed = -Constants.kp*xValueOff;// - Constants.minPower;
      }
    else if(Math.abs(xValueOff) <= 2)
      {
        positioningMovment = true;
        leftSpeed = Constants.kp*xValueOff + Constants.minPower;
        rightSpeed = -Constants.kp*xValueOff - Constants.minPower;
      }

    //Launches a power cell once in range
    if(Math.abs(xValueOff) <= 1.35 && launch && (distanceToPowerPort <= (23*12)||distanceToPowerPort <= (12*12)))
    {
      maxSpeed = .7;
      if(launchSpeed < maxSpeed)
      {
        launchSpeed += maxSpeed*windSpeed;//slowly increase the power to the shooter
      }
      if(timer.get() >= Constants.feedDelay)
      {
        //launcher.index(Constants.indexNEOSpeed);
        //launcher.feed(Constants.feedNEOSpeed);
      }
      //launcher.launch(launchSpeed);
    }
    else
    {
      timer.reset();
      launcher.stopLaunching();
    }

    //Moves into a set firing range
    if(Math.abs(xValueOff) <= 3 && positioningMovment && distanceToPowerPort > Constants.distanceToPowerportMaxIn)
    {
      leftSpeed = (.03 * (distanceToPowerPort - Constants.distanceToPowerportMaxIn)) + Constants.minPower;
      rightSpeed = (.03 * (distanceToPowerPort - Constants.distanceToPowerportMaxIn)) + Constants.minPower;
      SmartDashboard.putBoolean("Running", true);
    }
    else if(Math.abs(xValueOff) <= 3 && positioningMovment && distanceToPowerPort < Constants.distanceToPowerportMinIn)
    {
      leftSpeed = (-.03 * (Constants.distanceToPowerportMinIn - distanceToPowerPort)) - Constants.minPower;
      rightSpeed = (-.03 * (Constants.distanceToPowerportMinIn - distanceToPowerPort)) - Constants.minPower;
      SmartDashboard.putBoolean("Running", true);
    }
    else
    {
      positioningMovment = false;
      SmartDashboard.putBoolean("Running", false);
    }
    driveBase.move(ControlMode.PercentOutput , leftSpeed, rightSpeed);

    SmartDashboard.putNumber("Power", maxSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    RobotContainer.startTankDrive();
    limeL.vLEDoff();
    launcher.stopLaunching();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((!RobotContainer.driverLeftBumper.get())||(!RobotContainer.driverRightBumper.get()))
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
