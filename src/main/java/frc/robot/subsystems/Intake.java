/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    intakingNEO = new CANSparkMax(3, MotorType.kBrushless);
    intakingNEO.setInverted(false);
    articulationTalon = new TalonSRX(10);//positive is inward
    articulationTalon.setNeutralMode(NeutralMode.Brake);
    articulationTalon.configPeakOutputReverse(-1);
    articulationTalon.configClosedloopRamp(.2, 0);
    articulationTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    articulationTalon.config_kF(0, 0.15, 10);
		articulationTalon.config_kP(0, 1, 10);
		articulationTalon.config_kI(0, 0.00005, 10);
    articulationTalon.config_kD(0, 0.25, 10);
    //articulationTalon.setSelectedSensorPosition(0);
  }

  public void pivotIntake(double position)
  {
    //articulationTalon.set(ControlMode.PercentOutput, position);// change to position output once value is recorded
    articulationTalon.set(ControlMode.Position, position);
  }

  public void intakeCells(double speed)
  {
    intakingNEO.set(speed);
  }

  public int getPivotEncoderVaule()
  {
    return articulationTalon.getSelectedSensorPosition();
  }

  public void resetPivotEncoder()
  {
    articulationTalon.setSelectedSensorPosition(0);
  }

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
