// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SparkMaxPosition;

public class Amptake extends SubsystemBase 
{
  // Creates motors
  CANSparkMax pivot1;
  CANSparkMax pivot2;
  CANSparkMax intake;
  CANSparkMax belt;
  DigitalInput breakbeam;
  
  
  public Amptake() 
  {
    pivot1 = new CANSparkMax(0, MotorType.kBrushless);
    pivot2 = new CANSparkMax(0, MotorType.kBrushless);
    intake = new CANSparkMax(0, MotorType.kBrushless);
    belt = new CANSparkMax(0, MotorType.kBrushless);
    pivot2.follow(pivot1);
    pivot2.setInverted(true);
    breakbeam = new DigitalInput(0);
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
    ,this);
  }

  // Spit out a ring out the front 
  public Command spitOut()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(-1)),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0))
    );
  }

  // When the amp pivots are up it is used to place in amp
  public Command inAmp()
  {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intake.set(1)),
      new WaitCommand(1),
      new InstantCommand(() -> intake.set(0))
    );
  }

  // Move the note to the shooter
  public Command prepShooter()
  {
    return new InstantCommand(() -> belt.set(1));
  }

  // Move the pivot to a position
  public Command pivotTo(int position)
  {
    return new SparkMaxPosition(pivot1, position, 0, 50, this);
  }

  // Move pivot
  public Command ampPivot()
  {
    return new InstantCommand(() -> pivot1.set(1));
  }
  
  // Turn off the intake
  public Command off()
  {
    return new InstantCommand(() -> intake.set(0));
  }
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
