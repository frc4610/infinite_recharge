/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  private TalonSRX climberTalon;
  private double peak;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    peak = 1;
    climberTalon = new TalonSRX(2);
    RobotContainer.initMotor(climberTalon, peak);
  }

  public void move(double speed)
  {
    climberTalon.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
