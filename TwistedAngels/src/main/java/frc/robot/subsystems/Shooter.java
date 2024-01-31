// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public CANSparkMax shooter1;
  public CANSparkMax shooter2;


  public final int SHOOT_SPEED = 123456789;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkMax(0, MotorType.kBrushless);
    shooter2 = new CANSparkMax(0, MotorType.kBrushless);
    shooter2.setInverted(true);
    SmartDashboard.putData(this);
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

            configShooterMotor(shooter1);

            Thread.sleep(1000);
            LogOrDash.checkRevError("shooter motor 2 BURN",
                shooter2.burnFlash());
            Thread.sleep(1000);
        }

        catch(InterruptedException e)
        {
            DriverStation.reportError("Main thread interrupted while flashing swerve module!", e.getStackTrace());
        }
    }

    private void configShooterMotor(CANSparkMax m)
    {
      LogOrDash.checkRevError("shooter current limit", m.setSmartCurrentLimit(70));

      LogOrDash.checkRevError("shooter brakes", m.setIdleMode(IdleMode.kCoast));

      LogOrDash.checkRevError("shooter open loop ramp rate", m.setOpenLoopRampRate(0.2));

    }


  @Override
  public void periodic() {
    LogOrDash.sparkMaxDiagnostics("shooter/shooter1", shooter1);
    LogOrDash.sparkMaxDiagnostics("shooter/shooter2", shooter2);
  }

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

  public boolean readyShoot() {
    return shooter1.getEncoder().getVelocity() < SHOOT_SPEED * 0.95;
  }


}
