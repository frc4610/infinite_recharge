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
    encoderL = new Encoder(8, 7, 6, true);
    encoderR = new Encoder(1, 2, 3, false);
    encoderL.setDistancePerPulse(0.0092);
    encoderR.setDistancePerPulse(0.0092);
    //8192 counts per revolution
    //2048 cycles (1 Cycle = 1 Pulse) per revolution
    //18.85 inches per revolution (6 inch wheels)
    //Distance per pulse= 0.0092 inches (6 inch wheels)
    //12.57 inches per revolution (4 inch wheels)
    //Distance per pulse= 0.0061 inches (4 inch wheels)
  }
  public double getDistanceLeft(){
    return encoderL.getDistance();
  }

  public double getDistanceRight(){
    return encoderR.getDistance();
  }

  public void resetencoderL(){
    encoderL.reset();
  }

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
