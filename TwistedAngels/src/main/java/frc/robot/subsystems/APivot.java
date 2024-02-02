// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.robot.commands.SparkMaxPosition;

public class APivot extends SubsystemBase {
  
  public CANSparkMax pivot1;
  public CANSparkMax pivot2;
  public final int UP_POSITION = 1;

  public APivot() 
  {
    pivot1 = new CANSparkMax(51, MotorType.kBrushless);
    pivot2 = new CANSparkMax(52, MotorType.kBrushless);
    pivot2.follow(pivot1);
    pivot2.setInverted(true);
    SmartDashboard.putData(this);

    // Button to turn on/off sending debug data to the dashboard
    SmartDashboard.putData("Burn APivot Settings",  new InstantCommand(() -> configToFlash()));
  }

  public void configToFlash()
  {
      try
      {
          
          // shooter pivot motor 1
          LogOrDash.checkRevError("amp pivot motor 1 clear",
              pivot1.restoreFactoryDefaults());
          
          Thread.sleep(1000);

          configShooterPivotMotor(pivot1);
          LogOrDash.checkRevError("AmpPivotLimitF1", pivot1.setSoftLimit(SoftLimitDirection.kForward, 123456789));
          LogOrDash.checkRevError("AmpPivotLimitR1", pivot1.setSoftLimit(SoftLimitDirection.kReverse, 0));

          Thread.sleep(1000);
          LogOrDash.checkRevError("amp pivot motor 1 BURN",
              pivot1.burnFlash());
          Thread.sleep(1000);

          // shooter pivot motor 2
          LogOrDash.checkRevError("amp pivot motor 2 clear",
              pivot2.restoreFactoryDefaults());
          
          Thread.sleep(1000);

          configShooterPivotMotor(pivot2);

          Thread.sleep(1000);
          LogOrDash.checkRevError("amp pivot motor 2 BURN",
              pivot2.burnFlash());
          Thread.sleep(1000);
      }

      catch(InterruptedException e)
      {
          DriverStation.reportError("Main thread interrupted while flashing Amp Pivot!", e.getStackTrace());
      }
  }
  

  private void configShooterPivotMotor(CANSparkMax m)
  {
    LogOrDash.checkRevError("amp pivot current limit", m.setSmartCurrentLimit(40));

    LogOrDash.checkRevError("amp pivot brakes", m.setIdleMode(IdleMode.kBrake));

    LogOrDash.checkRevError("amp pivot open loop ramp rate", m.setOpenLoopRampRate(0.1));

  }


  public Command joystickMove(Supplier<Double> joystickM)
  {
    return new InstantCommand(() -> 
    pivot1.set(joystickM.get())
    ,this
    ).withName("joystickMove").repeatedly();
  }

  public Command pivotTo(int position)
  {
    return new SparkMaxPosition(pivot1, position, 0, 50, this).withName("pivotTo");
  }

  public Command pivotUp()
  {
    return pivotTo(UP_POSITION).withName("pivotUp");
    
  }

  public Command pivotDown()
  {
    return pivotTo(0).withName("pivotDown");
  }

  public boolean isDown() {
    return pivot1.getEncoder().getPosition() < UP_POSITION * 0.9;
  }




  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("aPivot/pivot1", pivot1);
    LogOrDash.sparkDiagnostics("aPivot/pivot2", pivot2);
    LogOrDash.logNumber("aPivot/pivot2/pivot1_position", pivot1.getEncoder().getPosition());
    LogOrDash.logNumber("aPivot/pivot2/pivot2_position", pivot2.getEncoder().getPosition());
  }
}
