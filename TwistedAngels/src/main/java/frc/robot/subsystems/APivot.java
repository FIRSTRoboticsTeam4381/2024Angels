// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;
import frc.robot.RobotContainer;
import frc.robot.commands.SparkMaxPosition;

public class APivot extends SubsystemBase {
  
  public CANSparkMax pivot1;
  public CANSparkMax pivot2;
  public final double UP_POSITION = 115;

  public APivot() 
  {
    pivot1 = new CANSparkMax(51, MotorType.kBrushless);
    pivot2 = new CANSparkMax(52, MotorType.kBrushless);
    
    pivot1.setInverted(false);

    SmartDashboard.putData(this);

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("aPivotUp", pivotUp());
    NamedCommands.registerCommand("aPivotDown", pivotDown());

    LogOrDash.setupSysIDTests(new SysIdRoutine.Config(),
     new CANSparkBase[]{pivot1}, new CANSparkBase[]{pivot1, pivot2}, this);

    SparkSaver.optimizeCANFrames(pivot1, true, false, true, false, false, false);
    SparkSaver.optimizeCANFrames(pivot2, false, false, false, false, false, false);

    SmartDashboard.putData("Configure APivot", new SparkSaver(pivot1, "pivot1", this)
      .setSmartCurrentLimit(40)
      .setBrakeMode()
      //.setOpenLoopRampRate(0.1)
      .setSoftLimits(0, 130)
      .configurePID(0, 0.80286, 0, 0.0032965, 0)
      .buildCommand()
      .andThen(new SparkSaver(pivot2, "pivot2", this)
      .setSmartCurrentLimit(40)
      .setBrakeMode()
      .follow(pivot1, true)
      .buildCommand()));

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
      //} else if (0.1 > Math.abs(joystickValue) ) {
      //  pivot1.set(0);
      } else {
        pivot1.set(joystickValue);
      }
      
    }
    ,this
    ).withName("joystickMove").repeatedly();
  }

  // Pivot the amp pivot to a positon
  public Command pivotTo(double position)
  {
    return new SparkMaxPosition(pivot1, position, 0, 0.5, this).withName("pivotTo");
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

  // Seeing if going up is safe with no collison
  public boolean isUpSafe() {
      return !RobotContainer.sPivot.isDanger() || pivot1.getEncoder().getPosition() > 85;
  }

  // Seeing if going down is safe with no collison
  public boolean isDownSafe() {
      return (!RobotContainer.sPivot.isDanger() 
        || pivot1.getEncoder().getPosition() < 30 
        || pivot1.getEncoder().getPosition() > 85);
        //&& !RobotContainer.hang.isDanger();
  }

  // If this is true then it should not go into a position of collison
  public boolean isDanger() {
      double p = pivot1.getEncoder().getPosition();  
      return 85 > p && 30 < p;
  }

  public boolean isHangDanger() {
      //double p = pivot1.getEncoder().getPosition();  
      //return p < 100;
      return false; //Current design doesn't block hang
  }

  @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("aPivot/pivot1", pivot1);
    LogOrDash.sparkDiagnostics("aPivot/pivot2", pivot2);
    LogOrDash.logNumber("aPivot/pivot1/position", pivot1.getEncoder().getPosition());
    LogOrDash.logNumber("aPivot/pivot2/position", pivot2.getEncoder().getPosition());

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
