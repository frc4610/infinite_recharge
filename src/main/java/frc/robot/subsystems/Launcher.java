/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  private final VictorSPX indexLeft;
  private final VictorSPX indexRight;
  private final VictorSPX feedController;
  private final CANSparkMax launcherLeft;
  private final CANSparkMax launcherRight;
  private final ColorSensorV3 ColorSensor;
  
  /**
   * Creates a new Launcher.
   * 
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

    ColorSensor = new ColorSensorV3(I2C.Port.kOnboard);  
    
  }

  public void index(final double speed) {
    indexLeft.set(ControlMode.PercentOutput, -(speed - .7));
    indexRight.set(ControlMode.PercentOutput, speed);
  }

  public void feed(final double speed) {
    feedController.set(ControlMode.PercentOutput, speed);
  }

  public void launch(final double speed) {
    launcherLeft.set(speed);
    launcherRight.set(speed);
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

  public int GetIR()
  {
    return ColorSensor.getIR();
  }

  public double GetLauncherSpeed()
  {
    return launcherLeft.getEncoder().getVelocity();

  }
}