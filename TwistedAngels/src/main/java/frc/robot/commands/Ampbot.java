// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.RobotContainer;

public class Ampbot extends ParallelCommandGroup {
  /** Creates a new Ampbot. */
  PIDController x;
  PIDController y;
  public Ampbot(Supplier<Double> forwardback, Supplier<Double> leftright, Supplier<Boolean> slow) {
    x = new PIDController(.005, 0, 0);
    y = new PIDController(.02, 0, 0);
    x.setTolerance(1);
    y.setTolerance(1);
    x.setSetpoint(0);
    y.setSetpoint(0);

    //addRequirements(RobotContainer.sPivot, RobotContainer.limelight, RobotContainer.s_Swerve, RobotContainer.shooter);
    addCommands(
      new TeleopSwerve(RobotContainer.s_Swerve, forwardback, leftright, this::getXPower, true, slow)

      );
  }

  public double getXPower()
  {
    if(RobotContainer.limelight.hasTarget())
    {
      return -x.calculate(RobotContainer.limelight.getX());
    }else{
      return 0;
    }

  }
}
