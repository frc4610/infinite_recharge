/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax climbTalon;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    climbTalon = new CANSparkMax(5, MotorType.kBrushless);

  }

  public void set(double speed)
  {
    climbTalon.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
