// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.function.Supplier;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;
import frc.robot.RobotContainer;
import frc.robot.commands.SparkMaxPosition;

public class Hang extends SubsystemBase {
 
 
 CANSparkMax hook1;
 CANSparkMax hook2;
 
  /** Creates a new Hang. */
  public Hang() {

    hook1 = new CANSparkMax(57, MotorType.kBrushless);
    hook2 = new CANSparkMax(58, MotorType.kBrushless);
    //hook2.follow(hook1,true);

    SmartDashboard.putData(this);

    SparkSaver.optimizeCANFrames(hook1, true, false, true, false, false, false);
    SparkSaver.optimizeCANFrames(hook2, false, false, false, false, false, false);

    SmartDashboard.putData("Configure Hang", new SparkSaver(hook1, "hook1", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      .setOpenLoopRampRate(0.1)
      .setSoftLimits(-60, 0)
      .buildCommand()
      .andThen(new SparkSaver(hook2, "hook2", this)
      .setSmartCurrentLimit(60)
      .setBrakeMode()
      .follow(hook1, true)
      .buildCommand()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    LogOrDash.sparkDiagnostics("hang/hook1", hook1);
    LogOrDash.sparkDiagnostics("hang/hook2", hook2);    
    LogOrDash.logNumber("hang/hook1/position", hook1.getEncoder().getPosition());
    LogOrDash.logNumber("hang/hook2/position", hook2.getEncoder().getPosition());

  }
  public Command hangTriggers(Supplier<Double> lt, Supplier<Double> rt){
    return new InstantCommand(() -> {
      if(isSafe()) {
        double r = (rt.get() +1 )/2.0;
        double l = (lt.get() +1)/2.0;
        double speed = r-l;
        hook1.set( speed);
      }

      else {
        hook1.set(0);
      }
    } ,
    this).repeatedly(); 
  }

  
  
  // Move the hang to "position" position
  public Command hangTo(int position) { // Set position for later commands
    return new SparkMaxPosition(hook1, position, 0, 50, this).withName("hangTo");
  }

  // Seeing if going up is safe with no collison
  public boolean isSafe() {
    return !RobotContainer.sPivot.isHangDanger() 
      && !RobotContainer.aPivot.isHangDanger() 
      || hook1.getEncoder().getPosition() > -3;
}

  public boolean isDanger() {
    double p = hook1.getEncoder().getPosition();  
    return p < -3;
  }

}
