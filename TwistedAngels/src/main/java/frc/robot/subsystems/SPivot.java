// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
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

    pivot1.setInverted(true);

    SparkSaver.optimizeCANFrames(pivot1, true, false, true, false, false, false);
    SparkSaver.optimizeCANFrames(pivot2, false, false, false, false, false, false);

    SmartDashboard.putData("Configure SPivot", new SparkSaver(pivot1, "pivot1", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      //.setOpenLoopRampRate(0.1)
      .setSoftLimits(0, 28)
      .configurePID(0, 7.0893, 0, 0.024286, 0)
      .buildCommand()
      .andThen(new SparkSaver(pivot2, "pivot2", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      .follow(pivot1, true)
      .buildCommand()));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("sPivotToShoot", pivotToShoot());
    NamedCommands.registerCommand("sPivotBack", pivotBack());
    NamedCommands.registerCommand("F3NoteShoot1", sPivotTo(24.2));
    NamedCommands.registerCommand("F3NoteShoot2", sPivotTo(28.3));
    NamedCommands.registerCommand("F3NoteShoot3", sPivotTo(24.6));
    NamedCommands.registerCommand("F3NoteShoot4", sPivotTo(21.4));
    NamedCommands.registerCommand("noBreakBeam", sPivotTo(21));
    NamedCommands.registerCommand("middleNotes", sPivotTo(17.8));
    NamedCommands.registerCommand("RedJustShootAngle", sPivotTo(20.3));


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
    }
  }

  // Basic Joystick control for the Shooter Pivot
  public Command joystickControl(Supplier<Double> joystickMove) { // JOYSTICK CONTROL FOR SHOOTER PIVOT
    return new InstantCommand( () -> 
    {
      double joystickValue = -joystickMove.get();
      if (joystickValue > 0 && !isUpSafe()) {
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

  // GO BACK TO REGULAR POSITION (0)
  public Command pivotBack() {
    return sPivotTo(0).withName("pivotBack");
  }

  // Seeing if going up is safe
  public boolean isUpSafe() {
      return !RobotContainer.aPivot.isDanger() || pivot1.getEncoder().getPosition() < 10;
  }

  // If not safe don't move  
  public boolean isDanger() {
    double p = pivot1.getEncoder().getPosition();  
    return 11 < p;
  }

  

}
