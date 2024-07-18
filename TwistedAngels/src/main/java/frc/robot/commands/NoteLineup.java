// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.LogOrDash;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NoteTracker;
import frc.robot.subsystems.Swerve;

public class NoteLineup extends Command {
  /** Creates a new NoteLineup. */
  NoteTracker notetracker;
  Swerve swerve;
  PIDController x;
  PIDController y;
  boolean invertY;
  public NoteLineup(Swerve s, NoteTracker nt, boolean i)  {
    // Use addRequirements() here to declare subsystem dependencies
    notetracker = nt;
    swerve = s;
    invertY = i;

    if(i) 
    {
      x = new PIDController(0.12, 0, 0);
      y = new PIDController(0.32, 0, 0);
    }
    else
    {
      x = new PIDController(0.1, 0, 0);
      y = new PIDController(0.23, 0, 0);
    }
    x.setTolerance(1.0);
    y.setTolerance(0.7);
    x.setSetpoint(0);
    y.setSetpoint(0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
    

   

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   PhotonTrackedTarget note = notetracker.getBestNote();
    if (note != null){
        double limeX = x.calculate(note.getYaw());
        double limeY = y.calculate(note.getPitch());
        if(invertY){
          limeY = limeY * -1;
        }
        //LogOrDash.logNumber("limeSpeeds/x", limeX);
        //LogOrDash.logNumber("limeSpeeds/y", limeY);
        swerve.drive(new Translation2d(limeY,limeX), 0, false, true);
    }else{
      swerve.drive(new Translation2d(0,0), 0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return notetracker.hasTarget() && x.atSetpoint() && y.atSetpoint();
  }
}
