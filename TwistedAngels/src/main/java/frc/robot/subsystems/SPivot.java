// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.LogOrDash;
import frc.robot.RobotContainer;
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
    SmartDashboard.putData("Burn SPivot Settings",  new InstantCommand(() -> configToFlash()).ignoringDisable(true));

    // Registering commands so that they can be accessed in Pathplanner
    NamedCommands.registerCommand("sPivotToShoot", pivotToShoot());
    NamedCommands.registerCommand("sPivotBack", pivotBack());

    LogOrDash.setupSysIDTests(new SysIdRoutine.Config(),
     new CANSparkBase[]{pivot1}, new CANSparkBase[]{pivot1, pivot2}, this);
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
    LogOrDash.logNumber("sPivot/pivot1/position", pivot1.getEncoder().getPosition());
    LogOrDash.logNumber("sPivot/pivot2/position", pivot2.getEncoder().getPosition());

    pivot1.getAppliedOutput();
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
      } else if (0.1 > Math.abs(joystickValue) ) {
        pivot1.set(0);
      } else {
        pivot1.set(joystickValue);
      }
    }
      ,this
    ).withName("JoystickControl").repeatedly();
  }

  // Make the shooter pivot go to a position
  public Command sPivotTo(int position) { // Set position for later commands
    return new SparkMaxPosition(pivot1, position, 0, 50, this).withName("sPivotTo");
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
      return !RobotContainer.sPivot.isDanger() || pivot1.getEncoder().getPosition() > 444;
  }

  // If not safe don't move  
  public boolean isDanger() {
    double p = pivot1.getEncoder().getPosition();  
    return 5 < p;
  }

}
