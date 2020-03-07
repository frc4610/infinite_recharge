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
  private double moveSpeed;
  private double error;
  private double P = 0.0212;
  private double I = .000212;
  private double integral = 0;
  private double setpoint;

  private boolean isAuto;
  private boolean stopLaunch;

  /**
   * Creates a new visionTarget.
   * 
   * @param plimeL The limeLight to pass to this command
   */
  public visionTarget(limeLight plimeL, DriveBase tdriveBase, Launcher tLauncher, navX tGyro, boolean Auto, boolean stopFlywheel) 
  {
    stopLaunch = stopFlywheel;
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
    error = 0;
    integral = 0;
    limeL.vLEDon();
    autoTimer.start();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    limeL.visionStoreValues();
    distanceToPowerPort = limeL.getDistance(Constants.groundToLimeLensIn, Constants.groundToPowerPortIn, Constants.groundToLimeLensRad);
    xValueOff = limeL.getXValueOff();
    error = xValueOff;
    integral += (error * .02);
    moveSpeed = (error * P) + (integral * I);
    SmartDashboard.putNumber("Distance to power port", distanceToPowerPort);
    SmartDashboard.putNumber("Vector to inner port", xValueOff);

    if(Math.abs(xValueOff) < 3 && Math.abs(xValueOff) > .8)
      {
        if(moveSpeed > 0)
        {
          moveSpeed += .02;
        }
        else
        {
          moveSpeed -= .02;
        }
        
      }
    driveBase.move(ControlMode.PercentOutput , moveSpeed, -moveSpeed);
    if(distanceToPowerPort < 125)
    {
      launchSpeed = .5;
      launcher.launch(launchSpeed);
      SmartDashboard.putNumber("Power Launch", launchSpeed);
    }
    else
    {
      launchSpeed = ((.0011043603*distanceToPowerPort) + .354096);
      launcher.launch(launchSpeed);
      SmartDashboard.putNumber("Power Launch", launchSpeed);
    }

    if((launcher.GetLauncherSpeed() + 200) >= Constants.launchMaxVelocity*launchSpeed && Math.abs(xValueOff) <= 4)
    {
      launcher.feed(Constants.feedNEOSpeed);
      launcher.index(Constants.indexNEOSpeed);
    }
    else 
    {
      launcher.feed(0);
      launcher.index(0);
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
      launcher.stopLaunching(stopLaunch);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((!RobotContainer.driverRightBumper.get()) && !isAuto)
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
