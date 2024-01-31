// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SPivot;
import frc.robot.subsystems.Swerve;
import frc.robot.RobotContainer;

public class ShootingMode extends Command {
  Swerve swerveDrive;
  SPivot pivot;
  Shooter shooter;
  RobotContainer robotContainer;
  CommandPS4Controller controller;


  public ShootingMode(Swerve mainDrive, SPivot sPivot, Shooter shooter) 
  {
    swerveDrive = mainDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mainDrive, pivot, shooter);
  }


  @Override
  public void initialize() 
  {
    shooter.shooterReady();
    //swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, robotContainer.driver, robotContainer.specialist, true));
  }

  @Override
  public void execute() 
  {
   
    

  }


  @Override
  public void end(boolean interrupted) 
  {
    //swerveDrive.setDefaultCommand(new TeleopSwerve(swerveDrive, robotContainer.driver, robotContainer.driver, true));
  }

 
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
