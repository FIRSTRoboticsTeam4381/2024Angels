// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    }

}
