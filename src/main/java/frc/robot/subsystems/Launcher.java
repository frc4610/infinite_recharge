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
    indexLeft.setInverted(true);
    indexRight = new CANSparkMax(4, MotorType.kBrushless);
    //indexRight.setIdleMode(IdleMode.kCoast);

    feedController = new CANSparkMax(8, MotorType.kBrushless);
    feedController.setInverted(true);

    launcherLeft = new CANSparkMax(3, MotorType.kBrushless);
    launcherRight = new CANSparkMax(2, MotorType.kBrushless);
    launcherRight.setInverted(true);
    //launcherRight.setIdleMode(IdleMode.kCoast);
    //launcherRight.follow(launcherLeft);
    
  }

  /**
   * Runs the index, with the left side slightly slower than the right
   * @param speed Speed in percent output to run the index
   */
  public void index(double speed)
  {
    indexLeft.set(speed-.1);
    indexRight.set(speed);
  }

  /**
   * Runs the feeder, try to wind up rather than set straight to the target
   * @param speed Speed in percent output to run the feeder
   */
  public void feed(double speed)
  {
    feedController.set(speed);
  }

  /**
   * Runs the launcher, try to wind up rather than set straight to the target
   * @param speed Speed in percent output to run the launcher
   */
  public void launch(double speed)
  {
    launcherLeft.set(speed);
    launcherRight.set(speed);
  }

  /**
   * Stops the index, feed, and launcher
   */
  public void stopLaunching()
  {
    indexRight.set(0);
    indexLeft.set(0);
    launcherLeft.set(0);
    launcherRight.set(0);
    feedController.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
