// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SPivot;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotContainer;


public class ShootingMode extends ParallelRaceGroup {
  

  public ShootingMode(CommandPS4Controller driver, CommandPS4Controller specialist)
  {
    addCommands(RobotContainer.shooter.shooterReady(),
     (new TeleopSwerve(RobotContainer.s_Swerve, 
        RobotContainer.interpolateJoystick (driver::getLeftY,0.05 ),
        RobotContainer.interpolateJoystick (driver::getLeftX,0.05),
        RobotContainer.interpolateJoystick(specialist::getRightX,0.05),
        true, 
        driver.R1()::getAsBoolean)),
      new ShootingLEDs(RobotContainer.shooter, RobotContainer.leds, RobotContainer.ledBuffer1, RobotContainer.led1));
    setName("shooterMode");
  }

}
