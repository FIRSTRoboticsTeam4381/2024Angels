// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;
import frc.robot.RobotContainer;
import frc.robot.commands.SparkMaxPosition;

public class SPivot extends SubsystemBase {
  private CANSparkMax pivot1;
  private CANSparkMax pivot2;


  /** Creates a new SPivot. */
  public SPivot() {
    pivot1 = new CANSparkMax(55, MotorType.kBrushless);
    pivot2 = new CANSparkMax(56, MotorType.kBrushless);
    //pivot2.follow(pivot1, true);
    SmartDashboard.putData(this);

    pivot1.setInverted(false);

    SparkSaver.optimizeCANFrames(pivot1, true, false, true, false, false, false);
    SparkSaver.optimizeCANFrames(pivot2, false, false, false, false, false, false);

    SmartDashboard.putData("Configure SPivot", new SparkSaver(pivot1, "pivot1", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      //.setOpenLoopRampRate(0.1)
      .setSoftLimits(0, 120)
      .configurePID(0, 1.1336, 0, 0.005024, 0)
      .buildCommand()
      .andThen(new SparkSaver(pivot2, "pivot2", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      .follow(pivot1, false)
      .buildCommand()));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("sPivotToShoot", pivotToShoot());
    NamedCommands.registerCommand("sPivotBack", pivotBack());
    //Front 3 auto sPivot #s
    NamedCommands.registerCommand("F3NoteShoot1", sPivotTo(117));
    NamedCommands.registerCommand("F3NoteShoot2", sPivotTo(80));
    NamedCommands.registerCommand("F3NoteShoot3", sPivotTo(106));
    NamedCommands.registerCommand("F3NoteShoot4", sPivotTo(95));
    //move sPivot out of the way of the break beam
    NamedCommands.registerCommand("noBreakBeam", sPivotTo(70));
    //Middle notes auto sPivot #s
    NamedCommands.registerCommand("middleNotes", sPivotTo(55));
    //Just backup and shoot angle
    NamedCommands.registerCommand("RedJustShootAngle", sPivotTo(60));
    //Ampside 3 notes auto sPivot angle #s
    NamedCommands.registerCommand("A3NoteShoot1", sPivotTo(110));
    NamedCommands.registerCommand("A3NoteShoot2", sPivotTo(47));
    NamedCommands.registerCommand("A3NoteShoot3", sPivotTo(48));
    NamedCommands.registerCommand("A3NoteShoot4", sPivotTo(45.6));
    //???
    NamedCommands.registerCommand("M2NoteShoot2", sPivotTo(110.2));
    NamedCommands.registerCommand("M2NoteShoot3", sPivotTo(44.53)); // No angle tested
    
    
    SmartDashboard.putData("disable sPivot lower limit  <<CAUTION>>", resetMode());


    LogOrDash.setupSysIDTests(new SysIdRoutine.Config(),
     new CANSparkBase[]{pivot1}, new CANSparkBase[]{pivot1, pivot2}, this);
  }


   @Override
  public void periodic() {
    LogOrDash.sparkDiagnostics("sPivot/pivot1", pivot1);
    LogOrDash.sparkDiagnostics("sPivot/pivot2", pivot2);    
    LogOrDash.logNumber("sPivot/pivot1/position", pivot1.getEncoder().getPosition());
    LogOrDash.logNumber("sPivot/pivot2/position", pivot2.getEncoder().getPosition());

    if (pivot1.getAppliedOutput() > 0 && !isUpSafe()) {
      this.getCurrentCommand().cancel();
      pivot1.set(0);
    } else if (pivot1.getAppliedOutput() < 0 && !isDownSafe()) {
      this.getCurrentCommand().cancel();
      pivot1.set(0);
    }
  }

  // Basic Joystick control for the Shooter Pivot
  public Command joystickControl(Supplier<Double> joystickMove) { // JOYSTICK CONTROL FOR SHOOTER PIVOT
    return new InstantCommand( () -> 
    {
      double joystickValue = -joystickMove.get();
      if (joystickValue > 0 && !isUpSafe()) {
        pivot1.set(0);
      } else if(joystickValue < 0 && !isDownSafe()) {
        pivot1.set(0);
      //} else if (0.1 > Math.abs(joystickValue) ) {
      //  pivot1.set(0);
      } else {
        pivot1.set(joystickValue);
      }
    }
      ,this
    ).withName("JoystickControl").repeatedly();
  }

  // Make the shooter pivot go to a position
  public Command sPivotTo(double position) { // Set position for later commands
    return new SparkMaxPosition(pivot1, position, 0, 0.2, this).withName("sPivotTo");
  }

  // Pivot to shooting position
  public Command pivotToShoot() {
    return sPivotTo(2).withName("pivotToShoot"); // Actual position is unknown as of 2/8/24
  }

  public Command pivotToCloseShot() {
    return sPivotTo(120).withName("pivotToShoot"); // Actual position is unknown as of 2/8/24
  }

  // GO BACK TO REGULAR POSITION (0)
  public Command pivotBack() {
    return sPivotTo(5).withName("pivotBack");
  }

  //pass
  public Command pass() {
    return sPivotTo(110).withName("pass");
  }

  public Command resetMode() {
    return new InstantCommand(() ->
    {
      pivot1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    });
  }

  // Seeing if going up is safe
  public boolean isUpSafe() {
    return !RobotContainer.aPivot.isDanger() || pivot1.getEncoder().getPosition() < 70;
  }

  public boolean isDownSafe() {
    return !RobotContainer.hang.isDanger() || pivot1.getEncoder().getPosition() > 4.5;
  } 

  // If not safe don't move  
  public boolean isDanger() {
    double p = pivot1.getEncoder().getPosition();  
    return p > 78;
  }

  // If not safe don't move hang
  public boolean isHangDanger() {
    double p = pivot1.getEncoder().getPosition();  
    return p < 120;
  }
}
