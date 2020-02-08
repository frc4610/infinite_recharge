/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    intakingNEO.setInverted(true);
    articulationTalon = new TalonSRX(7);//positive is inward
    articulationTalon.configPeakOutputReverse(-1);
    articulationTalon.configClosedloopRamp(.5, 0);
    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    articulationTalon.config_kF(0, 0.15, 10);
		articulationTalon.config_kP(0, 0.45, 10);
		articulationTalon.config_kI(0, 0.00005, 10);
    articulationTalon.config_kD(0, 0.25, 10);
    //articulationTalon.setSelectedSensorPosition(0);
  }

  /**
   * Moves the pivot to a specified encoder position
   * @param position The encoder position to get to the pivot
   */
  public void pivotIntake(double position)
  {
    articulationTalon.set(ControlMode.Position, position);
  }

  /**
   * Runs the intake
   * @param speed Speed to intake at
   */
  public void intakeCells(double speed)
  {
    intakingNEO.set(speed);
  }

  /**
   * Gets the value of the pivot encoder
   * @return The pivot encoder's raw position
   */
  public int getPivotEncoderVaule()
  {
    return articulationTalon.getSelectedSensorPosition();
  }

  /**
   * Resets the pivot encoder
   */
  public void resetPivotEncoder()
  {
    articulationTalon.setSelectedSensorPosition(0);
  }

  /**
   * Neutrals all the intake motors
   */
  public void neutralMotors()
  {
    articulationTalon.neutralOutput();
    intakingNEO.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
