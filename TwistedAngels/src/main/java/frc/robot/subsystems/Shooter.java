// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;

public class Shooter extends SubsystemBase {
  public CANSparkFlex shooter1;
  public CANSparkFlex shooter2;


  public final int SHOOT_SPEED = 4000;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkFlex(53, MotorType.kBrushless);
    shooter1.setInverted(true);
    shooter2 = new CANSparkFlex(54, MotorType.kBrushless);
    shooter2.setInverted(true);
    SmartDashboard.putData(this);

    SparkSaver.optimizeCANFrames(shooter1, false, true, false, false, false, false);
    SparkSaver.optimizeCANFrames(shooter2, false, true, false, false, false, false);

    SmartDashboard.putData("Configure Shooter", new SparkSaver(shooter1, "shooter1", this)
      .setSmartCurrentLimit(60)
      .setCoastMode()
      //.setOpenLoopRampRate(0.2)
      .buildCommand()
      .andThen(new SparkSaver(shooter2, "shooter2", this)
      .setSmartCurrentLimit(60)
      .setCoastMode()
      //.setOpenLoopRampRate(0.2)
      .buildCommand()));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("shooterReady", shooterReady());
  }

  public void configToFlash()
    {
        try
        {
            // shooter motor 1
            LogOrDash.checkRevError("shooter motor 1 clear",
                shooter1.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            configShooterMotor(shooter1);

            Thread.sleep(1000);
            LogOrDash.checkRevError("shooter motor 1 BURN",
                shooter1.burnFlash());
            Thread.sleep(1000);

            // shooter motor 2
            LogOrDash.checkRevError("shooter motor 2 clear",
                shooter2.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            configShooterMotor(shooter2);

            Thread.sleep(1000);
            LogOrDash.checkRevError("shooter motor 2 BURN",
                shooter2.burnFlash());
            Thread.sleep(1000);
        }

        catch(InterruptedException e)
        {
            DriverStation.reportError("Main thread interrupted while flashing shooter!", e.getStackTrace());
        }
    }

    private void configShooterMotor(CANSparkFlex m)
    {
      LogOrDash.checkRevError("shooter current limit", m.setSmartCurrentLimit(60));

      LogOrDash.checkRevError("shooter brakes", m.setIdleMode(IdleMode.kCoast));

      LogOrDash.checkRevError("shooter open loop ramp rate", m.setOpenLoopRampRate(0.2));

    }


  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("shooter/shooter1", shooter1);
    LogOrDash.sparkDiagnostics("shooter/shooter2", shooter2);
    LogOrDash.logNumber("shooter/shooter1/velocity", shooter1.getEncoder().getVelocity());
    LogOrDash.logNumber("shooter/shooter2/velocity", shooter2.getEncoder().getVelocity());
  }

  // Make the shooter get ready and spun up
  public Command shooterReady() {
    return new FunctionalCommand( () -> {

    }, () -> {
      if (SHOOT_SPEED > shooter1.getEncoder().getVelocity()) { // If the motor speed is not up to speed
        shooter1.set(1);
      } else { // Else
        shooter1.set(0);
      }

      if (SHOOT_SPEED > shooter2.getEncoder().getVelocity()) { // Same but for 2nd shooter motor
        shooter2.set(1);
      } else { // Else
        shooter2.set(0);
      }
    }, (isInterupted) -> {
      shooter1.set(0);
      shooter2.set(0);
    }, () -> {
      return false;
    }, this).withName("shooterReady");
    
  }

  // It is ready to shoot when true
  public boolean readyShoot() {
    return shooter1.getEncoder().getVelocity() < SHOOT_SPEED * 0.9;
  }


}
