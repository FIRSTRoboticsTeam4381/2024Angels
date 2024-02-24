// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public final class Autos {
  
  // TODO register commands in subsystem constructores using NamedCommands.registerCommand()
  // TODO use AutoBuilder.buildAutoChooser() to choose autos??

    // TODO test of full auto
    public static PreviewAuto testAuto(){
        return new PreviewAuto("Test");
        //return new WaitCommand(1);
    }

    // TODO test of a single path
    public static PreviewAuto testPath(){
        //PathPlannerPath path = PathPlannerPath.fromPathFile("TestPath1");

        //return AutoBuilder.followPathWithEvents(path);
        return new PreviewAuto(new WaitCommand(1));
    }

    /**
     * Blank Autonomous to be used as default dashboard option
     * @return Autonomous command
     */
    public static PreviewAuto none(){
        return new PreviewAuto(Commands.none());
    }

    // Gets the 3 notes in the front after shooting one
    public static PreviewAuto Front3Note(){
        return new PreviewAuto("Front3NoteAuto");
    }

    public static PreviewAuto Front3Note2(){
        return new PreviewAuto("Front3NoteAuto2");
    }

    public static PreviewAuto DefenseInAuto(){
        return new PreviewAuto("DefenseInAuto");
    }

    public static PreviewAuto middleNotesSelect(String autoName)
    {
        return new PreviewAuto(new SequentialCommandGroup(
            new PathPlannerAuto(autoName),
            new ConditionalCommand( new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()),
            new SequentialCommandGroup(
                new SelectCommand<Integer>(
                    Map.ofEntries(
                        Map.entry(1, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note1"))),
                        Map.entry(2, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note2"))),
                        Map.entry(3, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note3"))),
                        Map.entry(4, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note4"))),
                        Map.entry(5, AutoBuilder.followPath(PathPlannerPath.fromPathFile("Note5")))
                    ), Autos::chosenNote)
                // TODO Shooter to correct angle
                //RobotContainer.intake.toShoot()
            ), notesToGet::isEmpty).repeatedly()



        ), autoName);
    }

    public static PreviewAuto middleNotesCenter(){
        return middleNotesSelect("MiddleStart");
    }
    public static PreviewAuto middleNotesSource(){
        return middleNotesSelect("MiddleStart2");
    }

    public static int chosenNote() {
        return notesToGet.remove();
    }

    public static Queue<Integer> notesToGet = new LinkedList<>();

    public static void pickNotes() {
        String chooseNotes = SmartDashboard.getString("Choose Notes", "");
        notesToGet.clear(); 

        for(String n : chooseNotes.split(",")) {
            try {
                notesToGet.add(Integer.parseInt(n));
            }catch(Exception e){}
        }
    }
    
    public static class PreviewAuto {
        public Command auto;
        public ArrayList <Pose2d> preview = new ArrayList<>();


        public void showPreview() {
            if (preview != null) {
                RobotContainer.s_Swerve.field.getObject("path").setPoses(preview);
            }
        }

        public PreviewAuto(Command a) {
            auto = a;
        }

        public PreviewAuto(String s) {
            auto = new PathPlannerAuto(s);

            for(PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s))
            {
                preview.addAll(p.getPathPoses());
            }
        }

        public PreviewAuto(Command c, String s) {
            auto = c;

            for(PathPlannerPath p : PathPlannerAuto.getPathGroupFromAutoFile(s))
            {
                preview.addAll(p.getPathPoses());
            }
        }

    }

}
