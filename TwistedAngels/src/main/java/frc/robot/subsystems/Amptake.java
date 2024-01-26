// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Amptake extends SubsystemBase 
{
  // Creates motors
  CANSparkFlex pivot1;
  CANSparkFlex pivot2;
  CANSparkFlex intake;
  CANSparkFlex belt;
  
  
  public Amptake() 
  {
    pivot1 = new CANSparkFlex(0, MotorType.kBrushless);
    pivot2 = new CANSparkFlex(0, MotorType.kBrushless);
    intake = new CANSparkFlex(0, MotorType.kBrushless);
    belt = new CANSparkFlex(0, MotorType.kBrushless);
    pivot2.follow(pivot1);
    pivot2.setInverted(true);
  }

  // Intake on
  public Command Pickup()
  {
    return new ParallelCommandGroup(
      new InstantCommand(() -> intake.set(1))
    );
  }

  // Spit out a ring out the front 
  public Command SpitOut()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(-1)),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0))
    );
  }

  // When the amp pivots are up it is used to place in amp
  public Command InAmp()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(1)),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0))
    );
  }

  // Move the note to the shooter
  public Command PrepShooter()
  {
    return new InstantCommand(() -> belt.set(1));
  }

  // Put the amp bars up
  public Command AmpPivot()
  {
    return new InstantCommand(() -> pivot1.set(1));
  }
  
  // Turn off the intake
  public Command Off()
  {
    return new InstantCommand(() -> intake.set(0));
  }
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
