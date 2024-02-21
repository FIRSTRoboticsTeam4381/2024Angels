// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;

public class Shooter extends SubsystemBase {
  public CANSparkFlex shooter1;
  public CANSparkFlex shooter2;
  public CANSparkFlex conveyor;
  public CANSparkFlex conveyor2;


  public final int SHOOT_SPEED = -4000;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkFlex(53, MotorType.kBrushless);
    //shooter1.setInverted(true);
    shooter2 = new CANSparkFlex(54, MotorType.kBrushless);
    //shooter2.setInverted(true);
    conveyor = new CANSparkFlex(59, MotorType.kBrushless);

    conveyor2 = new CANSparkFlex(60, MotorType.kBrushless);

    SmartDashboard.putData(this);

    SparkSaver.optimizeCANFrames(shooter1, false, true, false, false, false, false);
    SparkSaver.optimizeCANFrames(shooter2, false, true, false, false, false, false);
    SparkSaver.optimizeCANFrames(conveyor, false, true, false, false, false, false);
    SparkSaver.optimizeCANFrames(conveyor2, false, true, false, false, false, false);

    SmartDashboard.putData("Configure Shooter", new SparkSaver(shooter1, "shooter1", this)
      .setSmartCurrentLimit(50, 15, 2000)
      .setCoastMode()
      //.setOpenLoopRampRate(0.2)
      .buildCommand()
      .andThen(new SparkSaver(shooter2, "shooter2", this)
      .setSmartCurrentLimit(50, 15, 2000)
      .setCoastMode()
      //.setOpenLoopRampRate(0.2)
      .buildCommand()
      .andThen(new SparkSaver(conveyor, "conveyor", this)
      .setSmartCurrentLimit(30)
      .setCoastMode()
      .buildCommand()
      .andThen(new SparkSaver(conveyor, "conveyor2", this)
      .setSmartCurrentLimit(30)
      .setCoastMode()
      //.follow(conveyor, false)
      .buildCommand()
      )
      )));
      

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("shooterReady", new ProxyCommand(shooterReady()));
  }


  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("shooter/shooter1", shooter1);
    LogOrDash.sparkDiagnostics("shooter/shooter2", shooter2);
    LogOrDash.sparkDiagnostics("shooter/conveyor", conveyor);
    LogOrDash.sparkDiagnostics("shooter/conveyor2", conveyor2);
    LogOrDash.logNumber("shooter/shooter1/velocity", shooter1.getEncoder().getVelocity());
    LogOrDash.logNumber("shooter/shooter2/velocity", shooter2.getEncoder().getVelocity());
    LogOrDash.logNumber("shooter/conveyor/velocity", conveyor.getEncoder().getVelocity());
    LogOrDash.logNumber("shooter/conveyor2/velocity", conveyor2.getEncoder().getVelocity());

    SmartDashboard.putBoolean("shooter/ready", readyShoot());
  }

  // Make the shooter get ready and spun up
  public Command shooterReady() {
    return new FunctionalCommand( () -> {
      conveyor.set(0.75);
      conveyor2.set(0.1);
    }, () -> {
      if (SHOOT_SPEED < shooter1.getEncoder().getVelocity()) { // If the motor speed is not up to speed
        shooter1.set(-1);
      } else { // Else
        shooter1.set(0);
      }

      if (SHOOT_SPEED < shooter2.getEncoder().getVelocity()) { // Same but for 2nd shooter motor
        shooter2.set(-1);
      } else { // Else
        shooter2.set(0);
      }

    }, (isInterupted) -> {
      shooter1.set(0);
      shooter2.set(0);
      conveyor.set(0);
      conveyor2.set(0);
    }, () -> {
      return false;
    }, this).withName("shooterReady");
    
  }

  // It is ready to shoot when true
  public boolean readyShoot() {
    return shooter1.getEncoder().getVelocity() < SHOOT_SPEED * 0.9 && 
      shooter2.getEncoder().getVelocity() < SHOOT_SPEED * 0.9;
  }




}
