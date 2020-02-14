/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }
public void setLED()
{
  for (var i = 0; i < ledBuffer.getLength(); i++) {
    ledBuffer.setHSV(i, 0, 100, 100);
  }
  led.setData(ledBuffer);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
