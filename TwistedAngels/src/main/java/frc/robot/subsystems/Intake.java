// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SparkMaxPosition;
import frc.robot.subsystems.Shooter;


public class Intake extends SubsystemBase 
{
  // Creates motors
  CANSparkMax intake;
  DigitalInput breakbeam;
  
  public Intake() 
  {
    intake = new CANSparkMax(50, MotorType.kBrushless);
    breakbeam = new DigitalInput(0);

    intake.setInverted(true);

    SmartDashboard.putData(this);

     // Button to turn on/off sending debug data to the dashboard
    //SmartDashboard.putData("Burn Intake Settings",  new InstantCommand(() -> configToFlash()).ignoringDisable(true));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("pickup", pickup());
    NamedCommands.registerCommand("inAmp", inAmp());
    NamedCommands.registerCommand("toShoot", toShoot());
    NamedCommands.registerCommand("intakeOff", off());

    SmartDashboard.putData("Configure Intake", new SparkSaver(intake, "intake", this)
      .setSmartCurrentLimit(30)
      .setBrakeMode()
      .buildCommand());
  }

  
  // Intake on
  public Command pickup()
  {
    //new InstantCommand(() -> intake.set(1))
    return new FunctionalCommand(
      () -> intake.set(1),
      () -> {},
      (isInterupted) -> intake.set(0),
      breakbeam::get
    ,this).withName("pickup");
  } 
  // Spit out a ring out the front 
  public Command spitOut()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(-0.3), this),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0), this)
    ).withName("spitOut");
  }

  // When the amp pivots are up it is used to place in amp 
  public Command inAmp()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(1), this),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0), this)
    ).withName("inAmp");
  }

  // Putting on belt to shoot
  public Command toShoot()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(0.3), this),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0), this)
    ).withName("toShoot");
  }
  
  
  // Turn off the intake
  public Command off()
  {
    return new InstantCommand(() -> intake.set(0), this).withName("off");
  }
  @Override
  
  public void periodic() 
  {
    LogOrDash.sparkDiagnostics("intake/motor", intake);
    LogOrDash.logNumber("intake/velocity", intake.getEncoder().getVelocity());
  }
}
