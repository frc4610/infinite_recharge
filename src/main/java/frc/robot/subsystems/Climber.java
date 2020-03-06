/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax climber;
  private CANPIDController climbPID;
  private CANEncoder climbEncoder;
  private DigitalInput limitSwitch;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    limitSwitch = new DigitalInput(0);
    climber = new CANSparkMax(5, MotorType.kBrushless);
    climber.setIdleMode(IdleMode.kBrake);
    climber.burnFlash();
    climbPID = climber.getPIDController();
    climbEncoder = climber.getEncoder();
    climbPID.setP(.05);
    climbPID.setI(.0);
    climbPID.setD(.0);
    climbPID.setOutputRange(-1, 1);
    climbEncoder.setPosition(-100);//safety measure to prevent the climb from shattering everything
  }

  public void set(double speed)
  {
    if(limitState() && speed > 0)
    climber.set(speed);
  }

  public boolean limitState()
  {
    return limitSwitch.get();
  }

  public double getEnc()
  {
    return climbEncoder.getPosition();
  }

  public void setEnc(double position)
  {
    climbPID.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
