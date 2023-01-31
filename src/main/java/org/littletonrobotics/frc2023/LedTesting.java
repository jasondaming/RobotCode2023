package org.littletonrobotics.frc2023;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C.Port;

public class LedTesting {
  AddressableLED led = new AddressableLED(0); //PWM port 0
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(60);

  public LedTesting(){
    led.setLength(ledBuffer.getLength());

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, 206, 38, 212);
      }
    led.setData(ledBuffer);
    led.start();
  }

  public void periodic() {}
}
