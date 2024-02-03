// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  
  AddressableLED m_led;
  AddressableLEDBuffer m_ledBuffer;
  int m_rainbowFirstPixelHue;

  public LEDs() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(150);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // This is the good rainbow
  public Command rainbow(AddressableLEDBuffer ledStrip) {

    return new InstantCommand(() -> {
      for (var i = 0; i < ledStrip.getLength(); i++) {
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledStrip.getLength())) % 180;
        ledStrip.setHSV(i, hue, 255, 128);
      }

      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(ledStrip);
    
      
    },this).repeatedly().ignoringDisable(true);

  }

  // The darker blue for the moving pattern
  public Command dark_blue_animated(AddressableLEDBuffer ledStrip) {
    return new InstantCommand(() -> {
      for (var i = 0; i < ledStrip.getLength(); i++) {

        final var hue = (m_rainbowFirstPixelHue + (i * 225 / ledStrip.getLength())) % 225;
  
        ledStrip.setHSV(i, 110,225, hue);
      }

      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 225;
      m_led.setData(ledStrip);
    
      
    },this).repeatedly();

  }

  public Command green(AddressableLEDBuffer ledStrip) {
    return new InstantCommand(() -> {
      for (var i = 0; i < ledStrip.getLength(); i++) {
        // Sets the specified LED to the HSV values for green
        ledStrip.setHSV(i, 60, 255, 255);
    }
    
    m_led.setData(ledStrip);
    },this);
  }

  public Command orange_animated(AddressableLEDBuffer ledStrip) {
    return new InstantCommand(() -> {
      for (var i = 0; i < ledStrip.getLength(); i++) {

        final var hue = (m_rainbowFirstPixelHue + (i * 225 / ledStrip.getLength())) % 225;
  
        ledStrip.setHSV(i, 5,255, hue);
      }

      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 225;
      m_led.setData(ledStrip);
    
      
    },this).repeatedly();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
