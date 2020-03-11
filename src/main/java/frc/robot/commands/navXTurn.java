/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.navX;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class navXTurn extends CommandBase {
  private double P = 0.00555;
  private double I = .00000555;
  private double integral = 0;
  private double setpoint;
  private double speed;
  private double error;
  private navX gyro;
  private DriveBase driveBase;
  private Timer timer;
  private boolean isAuto;

  /**
   * Creates a new navXTurn.
   */
  public navXTurn(navX Gyro, DriveBase tempdrive, double Setpoint, boolean auto){
    gyro = Gyro;
    driveBase = tempdrive;
    timer = new Timer();
    setpoint = Setpoint;
    isAuto = auto;
    addRequirements(tempdrive);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = 0;
    integral = 0;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setpoint - gyro.getYaw(); // Error = Target - Actual
    integral += (error * .02);
    speed = (P * error) + (I * integral); //Equation for power(rcw = power)
    driveBase.move(ControlMode.PercentOutput, speed, -speed);
  }

  // Called once the command ends or is interrupted.+
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    if(!isAuto)
    {
    RobotContainer.startTankDrive();
    }
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((!RobotContainer.driverXButton.get()&&!RobotContainer.driverYButton.get()&&!RobotContainer.driverBButton.get()) && !isAuto){
      return true;
    }
    else
    {
      return timer.get() > 1.75;
    }
  }
}
