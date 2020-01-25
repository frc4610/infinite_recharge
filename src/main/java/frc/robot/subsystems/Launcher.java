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

public class Launcher extends SubsystemBase {
  private CANSparkMax indexLeft;
  private CANSparkMax indexRight;
  private CANSparkMax feedController;
  private CANSparkMax launcherLeft;
  private CANSparkMax launcherRight;
  /**
   * Creates a new Launcher.
   */
  public Launcher() {
    indexLeft = new CANSparkMax(1, MotorType.kBrushless);
    indexRight = new CANSparkMax(4, MotorType.kBrushless);
    indexRight.follow(indexLeft);
    indexRight.setInverted(true);

    feedController = new CANSparkMax(0, MotorType.kBrushless);

    launcherLeft = new CANSparkMax(2, MotorType.kBrushless);
    launcherRight = new CANSparkMax(3, MotorType.kBrushless);
    launcherRight.follow(launcherLeft);
    launcherRight.setInverted(true);
  }

  public void index(double speed)
  {
    indexLeft.set(speed);
  }

  public void feed(double speed)
  {
    feedController.set(speed);
  }

  public void launch(double speed)
  {
    launcherLeft.set(speed);
  }

  public void stopLaunching()
  {
    feedController.set(0);
    indexLeft.set(0);
    launcherLeft.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
