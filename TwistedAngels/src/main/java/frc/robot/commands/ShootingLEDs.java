// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingLEDs extends Command {
  Shooter shooter;
  LEDs leds;
  AddressableLED aLED;
  AddressableLEDBuffer ledBuffer;

  public ShootingLEDs(Shooter s, LEDs l, AddressableLEDBuffer b, AddressableLED a) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = s;
    leds = l;
    aLED = a;
    ledBuffer = b;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new InstantCommand(() -> {
      for (var i = 0; i < ledBuffer.getLength(); i++) {

        final double position = (shooter.shooter1.getEncoder().getPosition() % ledBuffer.getLength());
        int pos = (int) Math.round(position);
        ledBuffer.setHSV(pos, 15 ,255, 255);
      }

      aLED.setData(ledBuffer);
    }
  ).repeatedly();
  }
}
