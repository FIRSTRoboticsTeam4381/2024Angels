// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LogOrDash;
import frc.robot.RobotContainer;
import frc.robot.commands.SparkMaxPosition;

public class APivot extends SubsystemBase {
  
  public CANSparkMax pivot1;
  public CANSparkMax pivot2;
  public final int UP_POSITION = 1000;

  public APivot() 
  {
    pivot1 = new CANSparkMax(51, MotorType.kBrushless);
    pivot2 = new CANSparkMax(52, MotorType.kBrushless);
    pivot2.follow(pivot1);
    pivot2.setInverted(true);
    SmartDashboard.putData(this);

    // Button to turn on/off sending debug data to the dashboard
    SmartDashboard.putData("Burn APivot Settings",  new InstantCommand(() -> configToFlash()).ignoringDisable(true));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("aPivotUp", pivotUp());
    NamedCommands.registerCommand("aPivotDown", pivotDown());

    LogOrDash.setupSysIDTests(new SysIdRoutine.Config(),
     new CANSparkBase[]{pivot1}, new CANSparkBase[]{pivot1, pivot2}, this);
  }

  public void configToFlash()
  {
      try
      {
          
          // amp pivot motor 1
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

          // amp pivot motor 2
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

  // Basic Joystick movement for the amp pivot
  public Command joystickMove(Supplier<Double> joystickM)
  {
    return new InstantCommand(() -> 
    {
      double joystickValue = -joystickM.get();
      if (joystickValue < 0 && !isDownSafe()) { // Checking if it is safe for the pivot to move
        pivot1.set(0);
      } else if (joystickValue > 0 && !isUpSafe()) {
        pivot1.set(0);
      } else if (0.1 > Math.abs(joystickValue) ) {
        pivot1.set(0);
      } else {
        pivot1.set(joystickValue);
      }
      
    }
    ,this
    ).withName("joystickMove").repeatedly();
  }

  // Pivot the amp pivot to a positon
  public Command pivotTo(int position)
  {
    return new SparkMaxPosition(pivot1, position, 0, 50, this).withName("pivotTo");
  }

  // Amp pivot up position
  public Command pivotUp()
  {
      return pivotTo(UP_POSITION).withName("pivotUp");
  }

  // Amp pivot down position
  public Command pivotDown()
  {
     return pivotTo(0).withName("pivotDown");
  }

  // Seeing if the pivot is down
  public boolean isDown() {
      return pivot1.getEncoder().getPosition() < UP_POSITION * 0.9; 
  }

  // Seeing if going down is safe with no collison
  public boolean isDownSafe() {
      return !RobotContainer.sPivot.isDanger() || pivot1.getEncoder().getPosition() < 5 || pivot1.getEncoder().getPosition() > 444;
  }

  // Seeing if going up is safe with no collison
  public boolean isUpSafe() {
      return !RobotContainer.sPivot.isDanger() || pivot1.getEncoder().getPosition() > 444;
  }

  // If this is true then it should not go into a position of collison
  public boolean isDanger() {
      double p = pivot1.getEncoder().getPosition();  
      return 444 > p && 5 < p;
  }


  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("aPivot/pivot1", pivot1);
    LogOrDash.sparkDiagnostics("aPivot/pivot2", pivot2);
    LogOrDash.logNumber("aPivot/pivot1/position", pivot1.getEncoder().getPosition());
    LogOrDash.logNumber("aPivot/pivot2/-position", pivot2.getEncoder().getPosition());

    // Cancels any movement if it is not safe
    if (pivot1.getAppliedOutput() > 0 && !isUpSafe()) { 
      this.getCurrentCommand().cancel();
      pivot1.set(0);
    } else if (pivot1.getAppliedOutput() < 0 && !isDownSafe()) {
      this.getCurrentCommand().cancel();
      pivot1.set(0);
    }
  }
}
