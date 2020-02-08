/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;





import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class encoder extends SubsystemBase { 
  private Encoder encoderL;
  private Encoder encoderR;
  /**
   * Creates a new encoder.
   */
  public encoder() {
    //A = Blue, B = Yellow, Index = Green
    encoderL = new Encoder(8, 7, false);
    encoderR = new Encoder(1, 2, true);
    encoderL.setDistancePerPulse(0.0096);
    encoderR.setDistancePerPulse(0.0096);
    encoderL.setSamplesToAverage(10);
    //8192 counts per revolution
    //2048 cycles (1 Cycle = 1 Pulse) per revolution
    //19.635 inches per revolution (6.25 inch wheels)
    //Distance per pulse= 0.0096 inches (6.25 inch wheels)
    //12.57 inches per revolution (4 inch wheels)
    //Distance per pulse= 0.0061 inches (4 inch wheels)
  }

  /**
   * Gets the distance in inches for the left drive encoder
   * @return The left distance in inches
   */
  public double getDistanceLeft(){
    return encoderL.getDistance();
  }

  /**
   * Gets the distance in inches for the right drive encoder
   * @return The right distance in inches
   */
  public double getDistanceRight(){
    return encoderR.getDistance();
  }

  /**
   * Resets left drive encoder
   */
  public void resetencoderL(){
    encoderL.reset();
  }

  /**
   * Resets right drive encoder
   */
  public void resetencoderR(){
    encoderR.reset();
  }

  /**
   * Determines if the drivebase is moving based on encoders
   * @return If the drivebase is moving
   */
  public boolean drivebaseIsMoving()
  {
    return encoderL.getRate() > .01 || encoderR.getRate() > .01;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
