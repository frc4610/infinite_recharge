/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private CANSparkMax intakingNEO;
  private TalonSRX articulationTalon;
  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakingNEO = new CANSparkMax(5, MotorType.kBrushless);
    articulationTalon = new TalonSRX(0);
    articulationTalon.setSelectedSensorPosition(0);
  }

  public void pivotIntake(double position)
  {
    articulationTalon.set(ControlMode.PercentOutput, position);// change to position output once value is recorded
  }

  public void intakeCells(double speed)
  {
    intakingNEO.set(speed);
  }

  public int getPivotEncoderVaule()
  {
    return articulationTalon.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
