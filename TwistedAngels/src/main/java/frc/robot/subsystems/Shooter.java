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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex shooter1;
  private CANSparkFlex shooter2;

  private CANSparkMax pivot1;
  private CANSparkMax pivot2;

  private final int SHOOT_SPEED = 123456789;

  /** Creates a new Shooter. */
  public Shooter() {
    shooter1 = new CANSparkFlex(0, MotorType.kBrushless);
    shooter2 = new CANSparkFlex(0, MotorType.kBrushless);

    pivot1 = new CANSparkMax(0, MotorType.kBrushless);
    pivot2 = new CANSparkMax(0, MotorType.kBrushless);

    pivot2.follow(pivot1, true);
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



            // shooter pivot motor 1
            LogOrDash.checkRevError("shooter pivot motor 1 clear",
                pivot1.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            configShooterPivotMotor(pivot1);
            LogOrDash.checkRevError("ShooterPivotLimitF1", pivot1.setSoftLimit(SoftLimitDirection.kForward, 123456789));
            LogOrDash.checkRevError("ShooterPivotLimitR1", pivot1.setSoftLimit(SoftLimitDirection.kReverse, 0));

            Thread.sleep(1000);
            LogOrDash.checkRevError("shooter pivot motor 1 BURN",
                pivot1.burnFlash());
            Thread.sleep(1000);

            // shooter pivot motor 2
            LogOrDash.checkRevError("shooter pivot motor 2 clear",
                pivot2.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            configShooterPivotMotor(pivot2);

            Thread.sleep(1000);
            LogOrDash.checkRevError("shooter pivot motor 2 BURN",
                pivot2.burnFlash());
            Thread.sleep(1000);
        }

        catch(InterruptedException e)
        {
            DriverStation.reportError("Main thread interrupted while flashing swerve module!", e.getStackTrace());
        }
    }

    private void configShooterMotor(CANSparkFlex m)
    {
      LogOrDash.checkRevError("shooter current limit", m.setSmartCurrentLimit(70));

      LogOrDash.checkRevError("shooter brakes", m.setIdleMode(IdleMode.kCoast));

      LogOrDash.checkRevError("shooter open loop ramp rate", m.setOpenLoopRampRate(0.2));

    }

    private void configShooterPivotMotor(CANSparkMax m)
    {
      LogOrDash.checkRevError("shooter pivot current limit", m.setSmartCurrentLimit(40));

      LogOrDash.checkRevError("shooter pivot brakes", m.setIdleMode(IdleMode.kBrake));

      LogOrDash.checkRevError("shooter pivot open loop ramp rate", m.setOpenLoopRampRate(0.1));

    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command spinUp() {
    return new FunctionalCommand( () -> {

    }, () -> {
      if (SHOOT_SPEED > shooter1.getEncoder().getVelocity()) {
        shooter1.set(1);
      } else {
        shooter1.set(0);
      }

      if (SHOOT_SPEED > shooter2.getEncoder().getVelocity()) {
        shooter2.set(1);
      } else {
        shooter2.set(0);
      }
    }, (isInterupted) -> {

    }, () -> {
      return false;
    }, this);
    
  }


}
