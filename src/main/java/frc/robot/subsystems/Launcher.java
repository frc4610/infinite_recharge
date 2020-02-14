/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

  private final CANSparkMax indexLeft;
  private final CANSparkMax indexRight;
  private final CANSparkMax feedController;
  private final CANSparkMax launcherLeft;
  private final CANSparkMax launcherRight;
  private final ColorSensorV3 ColorSensor;
  
  
 

  

  /**
   * Creates a new Launcher.
   * 
   *
   * 
   * 
   * 
   */
  public Launcher() {
    indexLeft = new CANSparkMax(1, MotorType.kBrushless);
    indexLeft.setInverted(true);
    indexRight = new CANSparkMax(4, MotorType.kBrushless);

    // indexRight.setIdleMode(IdleMode.kCoast);

    feedController = new CANSparkMax(8, MotorType.kBrushless);
    feedController.setInverted(true);

    launcherLeft = new CANSparkMax(3, MotorType.kBrushless);
    launcherRight = new CANSparkMax(2, MotorType.kBrushless);
    launcherRight.setInverted(true);
    ColorSensor = new ColorSensorV3(I2C.Port.kOnboard);  
    
  }

  public void index(final double speed) {
    indexLeft.set(speed - .1);
    indexRight.set(speed);
  }

  public void feed(final double speed) {
    feedController.set(speed);
  }

  public void launch(final double speed) {
    launcherLeft.set(speed);
    launcherRight.set(speed);
  }

  public void stopLaunching() {
    indexRight.set(0);
    indexLeft.set(0);
    launcherLeft.set(0);
    launcherRight.set(0);
    feedController.set(0);
  }
   


  

  @Override
  public void periodic() 
  {

  }
  public double GetIR()
  {
    return ColorSensor.getIR();
  }

}

