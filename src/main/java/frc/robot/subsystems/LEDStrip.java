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
  private double rainbowFirstPixel;
  private int hue;
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    rainbowFirstPixel = 0;
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(60);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }
public void setLEDRainbow()
{
  for (var i = 0; i < ledBuffer.getLength(); i++) 
  {
    hue = (int) ((rainbowFirstPixel + (i *180 / ledBuffer.getLength())) % 180);
    ledBuffer.setHSV(i, hue, 255, 100);
  }
  led.setData(ledBuffer);
  rainbowFirstPixel += 1;
  rainbowFirstPixel %= 180;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
