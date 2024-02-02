// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.robot.commands.SparkMaxPosition;

public class SPivot extends SubsystemBase {
  private CANSparkMax pivot1;
  private CANSparkMax pivot2;


  /** Creates a new SPivot. */
  public SPivot() {
    pivot1 = new CANSparkMax(55, MotorType.kBrushless);
    pivot2 = new CANSparkMax(56, MotorType.kBrushless);
    pivot2.follow(pivot1, true);
    SmartDashboard.putData(this);

    // Button to turn on/off sending debug data to the dashboard
    SmartDashboard.putData("Burn SPivot Settings",  new InstantCommand(() -> configToFlash()));
  }


  public void configToFlash()
  {
      try
      {
          
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
          DriverStation.reportError("Main thread interrupted while flashing Shooter Pivot!", e.getStackTrace());
      }
  }
  

  private void configShooterPivotMotor(CANSparkMax m)
  {
    LogOrDash.checkRevError("shooter pivot current limit", m.setSmartCurrentLimit(40));

    LogOrDash.checkRevError("shooter pivot brakes", m.setIdleMode(IdleMode.kBrake));

    LogOrDash.checkRevError("shooter pivot open loop ramp rate", m.setOpenLoopRampRate(0.1));

  }

  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("sPivot/pivot1", pivot1);
    LogOrDash.sparkDiagnostics("sPivot/pivot2", pivot2);    
  }

  public Command joystickControl(Supplier<Double> joystickMove) { // JOYSTICK CONTROL FOR SHOOTER PIVOT
    return new InstantCommand( () -> 
      pivot1.set(joystickMove.get()),
      this
    ).withName("JoystickControl").repeatedly();
  }

  public Command sPivotTo(int position) { // Set position for later commands
    return new SparkMaxPosition(pivot1, position, 0, 50, this).withName("sPivotTo");
  }

  public Command pivotToShoot() { // GO TO SHOOT POSITION
    return new InstantCommand(
     () -> sPivotTo(2)
    ).withName("pivotToShoot");
  }

   public Command pivotBack() { // GO BACK TO REGULAR POSITION (0)
    return new InstantCommand(
     () -> sPivotTo(0)
    ).withName("pivotBack");
  }
}
