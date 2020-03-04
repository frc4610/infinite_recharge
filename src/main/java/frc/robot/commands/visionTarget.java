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
import frc.robot.subsystems.navX;

public class visionTarget extends CommandBase {
  private limeLight limeL;
  private DriveBase driveBase;
  private Launcher launcher;
  private Timer timer;
  private Timer autoTimer;
  private navX gyro;

  private double distanceToPowerPort;
  private double xValueOff;
  private double launchSpeed;

  private boolean isAuto;

  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL, DriveBase tdriveBase, Launcher tLauncher, navX tGyro, boolean Auto) 
  {
    gyro = tGyro;
    driveBase = tdriveBase;
    limeL = plimeL;
    launcher = tLauncher;
    timer = new Timer();
    isAuto = Auto;
    autoTimer = new Timer();
    addRequirements(tLauncher);
    addRequirements(plimeL);
    launchSpeed = 0;
    addRequirements(tdriveBase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.start();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = limeL.getXValueOff();
    double error = xValueOff;
    error *=  Constants.kp;
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);

    /*if(Math.abs(xValueOff) > 2)
      {
        leftSpeed = Constants.kp*xValueOff;
        rightSpeed = -Constants.kp*xValueOff;
      }
    else if(Math.abs(xValueOff) <= 2)
      {
        leftSpeed = Constants.kp*xValueOff + Constants.minPower;
        rightSpeed = -Constants.kp*xValueOff - Constants.minPower;
      }*/
    driveBase.move(ControlMode.PercentOutput , error, -error);
    if(Math.abs(xValueOff) <= 3.5)
    {
      if(distanceToPowerPort < 196)
      {
        launchSpeed = ((.03573762578441*distanceToPowerPort) + 43.595453195203)/100;
        //launchSpeed = Constants.baselineLaunchSpeedLower + (distanceToPowerPort / 2400);
        launcher.launch(launchSpeed);
        SmartDashboard.putNumber("Power Launch", launchSpeed);
      }
      else
      {
        launchSpeed = ((.202377876*distanceToPowerPort) + 12.04764524)/100;
        //launchSpeed = Constants.baselineLaunchSpeedHigher + (distanceToPowerPort / 677.277);
        launcher.launch(launchSpeed);
        SmartDashboard.putNumber("Power Launch", launchSpeed);
      }
      if(timer.get() >= Constants.feedDelay)
      {
        launcher.index(Constants.indexNEOSpeed);
        if((launcher.GetLauncherSpeed() + 175) >= Constants.launchMaxVelocity*launchSpeed)
        {
          launcher.feed(Constants.feedNEOSpeed);
        }
        else 
        {
          launcher.feed(0);
        }
      }
    }
    else
    {
      timer.reset();
      launcher.stopLaunching();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    limeL.vLEDoff();
    if(!isAuto)
    {
      RobotContainer.startTankDrive();
      RobotContainer.startManualLaunch();
    }
    else{
      limeL.vLEDoff();
      launcher.stopLaunching();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(((!RobotContainer.driverLeftBumper.get())||(!RobotContainer.driverRightBumper.get())) && !isAuto)
    {
      return true;
    }
    else if(isAuto && autoTimer.get() >= Constants.autoLaunchDelay)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
