package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command{
    
    private double rotation;
    private Translation2d translation;
    private boolean openLoop;

    private Swerve s_Swerve;
    //private CommandPS4Controller controller;
    //private CommandPS4Controller controller2;
    private Supplier<Double> forward;
    private Supplier<Double> leftright;
    private Supplier<Double> rotate;
    private Supplier<Double> slow;

    
    /*
     * Driver Control command
     * @param s_Swerve Swerve subsystem
     * @param controller PS4 controller
     * @param openLoop True
     */
    public TeleopSwerve(Swerve s_Swerve, Supplier<Double> forward, Supplier<Double> leftright, Supplier<Double> rotate, boolean openLoop, Supplier<Double> slow){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.forward = forward;
        this.leftright = leftright;
        this.rotate = rotate;
        this.openLoop = openLoop;
        this.slow = slow;

    }

    @Override
    public void execute(){
        double yAxis = -forward.get();
        double xAxis = -leftright.get();
        double rAxis = -rotate.get();

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        /* Slow Trigger */
        double slowdown = 1 - (slow.get() < Constants.stickDeadband ? 0 : slow.get());
        yAxis *= slowdown;
        xAxis *= slowdown;
        rAxis *= slowdown;

        /* Calculates inputs for swerve subsystem */
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, openLoop);

    }
}
