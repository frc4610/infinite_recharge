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

  private boolean isAuto;

  private boolean previousState;
  private Timer feedTimer;

  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL, DriveBase tdriveBase, Launcher tLauncher, boolean Auto) 
  {
    leftSpeed = 0;
    rightSpeed = 0;
    driveBase = tdriveBase;
    limeL = plimeL;
    launcher = tLauncher;
    timer = new Timer();
    isAuto = Auto;
    addRequirements(tLauncher);
    addRequirements(plimeL);
    launchSpeed = 0;
    windSpeed = Constants.windSpeedNEO;
    feedTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = -limeL.getXValueOff();
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);

    if(xValueOff > .75)
      {
        leftSpeed = (Constants.kp*xValueOff) - Constants.minPower;
        rightSpeed = -(Constants.kp*xValueOff) + Constants.minPower;
      }
    else if(xValueOff < .75)
      {
        leftSpeed = (Constants.kp*xValueOff) + Constants.minPower;
        rightSpeed = -(Constants.kp*xValueOff) - Constants.minPower;
      }

    if(Math.abs(xValueOff) <= 3.5)
    {
      maxSpeed = .7;
      if(launchSpeed < maxSpeed)
      {
        launchSpeed += maxSpeed*windSpeed;//slowly increase the power to the shooter
      }
      if(timer.get() >= Constants.feedDelay)
      {
        launcher.index(Constants.indexNEOSpeed);
        if(RobotContainer.stateOfFeed() && !previousState)
        {
          feedTimer.start();
        }
        else if (!RobotContainer.stateOfFeed() && previousState)
        {
          feedTimer.reset();
        }
        else if(!RobotContainer.stateOfFeed() && !previousState)
        { 
          feedTimer.start();
        }

        previousState = RobotContainer.stateOfFeed();
        
        if(feedTimer.get() >= .25)
        {
         launcher.feed(Constants.feedNEOSpeed);
        }
        else
        {
          launcher.feed(0);
        }
        launcher.feed(Constants.feedNEOSpeed);
      }
      if(distanceToPowerPort < 192)
      {
        launcher.launch(Constants.baselineLaunchSpeedLower + (distanceToPowerPort / 2400));
      }
      else
      {
        launcher.launch(Constants.baselineLaunchSpeedHigher + (distanceToPowerPort / 2124));
      }
    }
    else
    {
      timer.reset();
      launcher.stopLaunching();
    }
    driveBase.move(ControlMode.PercentOutput , leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    if(!isAuto)
    {
      RobotContainer.startTankDrive();
    }
    limeL.vLEDoff();
    launcher.stopLaunching();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(((!RobotContainer.driverLeftBumper.get())||(!RobotContainer.driverRightBumper.get())) && !isAuto)
    {
      return true;
    }
    else if(isAuto && timer.get() >= Constants.autoLaunchDelay)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
