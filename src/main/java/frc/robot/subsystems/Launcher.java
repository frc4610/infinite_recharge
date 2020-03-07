/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase {
  private  VictorSPX indexLeft;
  private  VictorSPX indexRight;
  private  VictorSPX feedController;
  private  CANSparkMax launcherLeft;
  private  CANSparkMax launcherRight;
  private  CANPIDController PIDcontrollerL;
  private  CANPIDController PIDcontrollerR;
  private  CANEncoder encoderL;
  private  CANEncoder encoderR;
  
  
    
  
  /**
   * Creates a new Launcher.
   * 
   *
   * 
   
   * 
   */
  public Launcher() {
    indexLeft = new VictorSPX(13);
    indexLeft.setInverted(true);
    indexRight = new VictorSPX(12);

    // indexRight.setIdleMode(IdleMode.kCoast);

    feedController = new VictorSPX(11);
    feedController.setInverted(false);

    launcherLeft = new CANSparkMax(8, MotorType.kBrushless);
    launcherRight = new CANSparkMax(9, MotorType.kBrushless);
    launcherLeft.setOpenLoopRampRate(2);
    launcherRight.setOpenLoopRampRate(2);
    launcherRight.setClosedLoopRampRate(2);
    launcherLeft.setClosedLoopRampRate(2);
    launcherLeft.setIdleMode(IdleMode.kBrake);
    launcherRight.setIdleMode(IdleMode.kBrake);
    launcherRight.setInverted(true);
    launcherLeft.burnFlash();
    launcherRight.burnFlash();

    PIDcontrollerL =  launcherLeft.getPIDController();
    PIDcontrollerR =  launcherRight.getPIDController();
    encoderL = launcherLeft.getEncoder();
    encoderR = launcherRight.getEncoder();
    PIDcontrollerL.setFF(Constants.launchF);
    PIDcontrollerR.setFF(Constants.launchF);
    PIDcontrollerL.setP(Constants.launchP);//Ku.45
    PIDcontrollerR.setP(Constants.launchP);
    PIDcontrollerL.setD(Constants.launchD);
    PIDcontrollerR.setD(Constants.launchD);
    PIDcontrollerL.setI(Constants.launchI);//.54Ku/T
    PIDcontrollerR.setI(Constants.launchI);
    PIDcontrollerL.setOutputRange(0, .8);
    PIDcontrollerR.setOutputRange(0, .8);
  }

  public void index(final double speed) {
    indexLeft.set(ControlMode.PercentOutput, speed - .4);
    indexRight.set(ControlMode.PercentOutput, speed);
  }

  public void feed(final double speed) {
    feedController.set(ControlMode.PercentOutput, speed);
  } 

  public void launch(final double Speed) {
    PIDcontrollerL.setReference(Constants.launchNEOSpeed * Constants.launchMaxVelocity, ControlType.kVelocity);
    PIDcontrollerR.setReference(Constants.launchNEOSpeed * Constants.launchMaxVelocity, ControlType.kVelocity);
  }

  public void stopLaunching() {
    indexRight.set(ControlMode.PercentOutput, 0);
    indexLeft.set(ControlMode.PercentOutput, 0);
    launcherLeft.set(0);
    launcherRight.set(0);
    feedController.set(ControlMode.PercentOutput, 0);
  }
   
  @Override
  public void periodic() 
  {

  } 

  public double GetLauncherSpeed()
  {
    return launcherLeft.getEncoder().getVelocity();

  }
 public void maxSpeed(double MaxSpeed)
 {
  PIDcontrollerL.setOutputRange(-1, MaxSpeed);
  PIDcontrollerR.setOutputRange(-1, MaxSpeed);
 }
 public void stopIndex()
 {
  indexRight.set(ControlMode.PercentOutput, 0);
  indexLeft.set(ControlMode.PercentOutput, 0);
 }
}