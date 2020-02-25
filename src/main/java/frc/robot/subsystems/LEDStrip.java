/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private double rainbowFirstPixel;
  private int pulseFirstPixel;
  private int hue;
  private int pixel;
  /**
   * Creates a new LEDStrip.
   */
  public LEDStrip() {
    rainbowFirstPixel = 0;
    pulseFirstPixel = 0;
    led = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(50);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

/**
 * Sets LEDs to a solid color 
 * @param setToHue Hue to set: hue 0 is red, 120 is green, and 240 is blue
 */
public void setLEDSolid(int setToHue)
{
  for (var i = 0; i < ledBuffer.getLength(); i++) 
  {
    ledBuffer.setHSV(i, setToHue, 255, 100);
  }
  led.setData(ledBuffer);
}

/**
 * Sets LEDS to a pulse of color, white in between pulses
 * @param setToHue Hue to set: hue 0 is red, 60 is green, and 120 is blue (Half hue values)
 * @param pulseLength Pusle length: input the amount of colored pixels before a white pixel appears
 */
public void setLEDPulse(int setToHue, int pulseLength){
  for (var i = 0; i < 50 + pulseFirstPixel; i++) 
  {
    pixel = i + pulseFirstPixel;
    pixel %= 50;
    if(pixel % (pulseLength + 1) == 0)
    {
      ledBuffer.setHSV((i%50), setToHue, 0, 100);
    }
    else
    {
      ledBuffer.setHSV((i%50), setToHue, 255, 100);
    }
  }
  led.setData(ledBuffer);
  pulseFirstPixel += 1;
  pulseFirstPixel %= 50;
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
