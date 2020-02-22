/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax climber;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climber = new CANSparkMax(5, MotorType.kBrushless);
    climber.setIdleMode(IdleMode.kBrake);
    climber.burnFlash();
  }

  public void set(double speed)
  {
    climber.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
