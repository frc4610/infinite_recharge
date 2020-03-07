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
  private boolean setLimit;
  /**
   * Creates a new Climber.
   */
  public Climber() {
    limitSwitch = new DigitalInput(5);
    climber = new CANSparkMax(5, MotorType.kBrushless);
    climber.setIdleMode(IdleMode.kBrake);
    climber.setInverted(true);
    climber.burnFlash();
    climbPID = climber.getPIDController();
    climbEncoder = climber.getEncoder();
    climbPID.setP(.05);
    climbPID.setI(.0);
    climbPID.setD(.0);
    climbPID.setOutputRange(-1, 1);
    climbEncoder.setPosition(0);//safety measure to prevent the climb from shattering everything
  }

  public void set(double speed)
  {
    if(limitState())
    {
      limitSetup(true);
    }
    double moveSpeed = speed;
    if(climbEncoder.getPosition() <= 10 && moveSpeed < -.3)
    {
      moveSpeed = -.3;
    }
    if(!(limitState() && speed < 0))
    {
      climber.set(moveSpeed);
    }
    else
    {
      climber.set(0);
    }
  }

  public boolean limitState()
  {
    return limitSwitch.get();
  }

  public double getEnc()
  {
    if(limitState())
    {
      limitSetup(true);
    }
    if(setLimit)
    {
      return climbEncoder.getPosition();
    }
    else
    {
      return 25;
    }
  
  
  }

  public void setEnc(double position)
  {
    if(limitState())
    {
      limitSetup(true);
    }
    climbPID.setReference(position, ControlType.kPosition);
  }

  public void limitSetup(boolean setup)
  {
    if(setup && !setLimit)
    {
      climbEncoder.setPosition(0);
    }
    setLimit = setup;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
