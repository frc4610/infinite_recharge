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
  double P = 0.0075;
  double setpoint;
  navX gyro;
  DriveBase driveBase;
  private double rcw;
  private Timer timer;
  private boolean isAuto;

  /**
   * Creates a new navXTurn.
   */
  public navXTurn(navX gyro, DriveBase tempdrive, double Setpoint, boolean auto){
    this.gyro = (navX) gyro;
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
    timer.start();
  }

  public void PID() {
    double error = setpoint - navX.getYaw(); // Error = Target - Actual
    this.rcw = (P * error); //Equation for power(rcw = power)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PID();
    driveBase.move(ControlMode.PercentOutput, rcw, -rcw);
  }

  // Called once the command ends or is interrupted.+
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
    RobotContainer.startTankDrive();
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((!RobotContainer.driverXButton.get()&&!RobotContainer.driverYButton.get()&&!RobotContainer.driverBButton.get()) && !isAuto){
      return true;
    }
    else
    {
      return timer.get() > 1;
    }
  }
}
