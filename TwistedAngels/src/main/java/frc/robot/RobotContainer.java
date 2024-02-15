// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.LogOrDash;
import frc.robot.autos.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.APivot;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SPivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Limelight;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootingMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    public CommandPS4Controller driver = new CommandPS4Controller(0);
    public CommandPS4Controller specialist = new CommandPS4Controller(1);

    /* Driver Buttons */
    private final Trigger zeroSwerve = driver.options();

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static APivot aPivot;
    public static Intake intake;
    public static Hang hang;
    public static Shooter shooter;
    public static SPivot sPivot;
    public static LEDs leds;
    public static ShootingMode shooterMode;
    public static AddressableLED led1;
    public static AddressableLEDBuffer ledBuffer1;
    public static Limelight limelight;

    //Auto Chooser
    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
          interpolateJoystick(driver::getLeftY,0.05),
          interpolateJoystick(driver::getLeftX,0.05), 
          interpolateJoystick (driver::getRightX,0.05),
             true, driver::getR2Axis));
        aPivot = new APivot();
        intake = new Intake();
        hang = new Hang();
        shooter = new Shooter();
        sPivot = new SPivot();
        leds = new LEDs();
        limelight = new Limelight();
        shooterMode = new ShootingMode(driver, specialist);
        //led1 = new AddressableLED(2);
        ledBuffer1 = new AddressableLEDBuffer(10);
        
        aPivot.setDefaultCommand(aPivot.joystickMove(interpolateJoystick(specialist::getLeftY, 0.05)));
        sPivot.setDefaultCommand(sPivot.joystickControl(interpolateJoystick(specialist::getRightY, 0.05)));
        hang.setDefaultCommand(hang.hangTriggers(specialist::getL2Axis, specialist::getR2Axis));

        // Configure the button bindings
        configureButtonBindings();

        // Add autonomous options to chooser
        m_AutoChooser.setDefaultOption("None", Autos.none());
        // TODO m_AutoChooser.addOption("PathPlanner Example", Autos.exampleAuto());
        m_AutoChooser.addOption("Test", Autos.testAuto());
        m_AutoChooser.addOption("3NoteFront", Autos.Front3Note());

        SmartDashboard.putData("Choose Auto:", m_AutoChooser);

    }

    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
    private void configureButtonBindings(){
        // Button to reset swerve odometry and anglesssss
        zeroSwerve
            .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())
            .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
        driver.cross().toggleOnTrue(shooterMode);
        specialist.R1().onTrue(new SequentialCommandGroup (
             intake.toShoot(),
             new WaitCommand(1.5),
             new InstantCommand( () -> {
                Command c = shooter.getCurrentCommand();
                if(c != null)
                    c.cancel();
            }),
             sPivot.pivotBack()
             ).unless(shooter::readyShoot).withName("shoot"));
        specialist.L1().onTrue(intake.inAmp().unless(aPivot::isDown).withName("scoreInAmp"));

        specialist.povUp().onTrue(aPivot.pivotUp().withName("aPivotUp"));
        specialist.povDown().onTrue(aPivot.pivotDown().withName("aPivotDown"));

        specialist.circle().toggleOnTrue(intake.pickup().withName("pickup"));
        specialist.cross().onTrue(intake.spitOut().withName("spitOut"));

        specialist.PS().onTrue(new InstantCommand(() -> { // Cancel all commands
            try {
                intake.getCurrentCommand().cancel();
            } catch(NullPointerException e) {
                //nothing
            } try {
                aPivot.getCurrentCommand().cancel();
            } catch(NullPointerException e) {
                //nothing
            } try {
                sPivot.getCurrentCommand().cancel();
            } catch(NullPointerException e) {
                //nothing
            } try {
                shooter.getCurrentCommand().cancel();
            } catch(NullPointerException e) {
                //nothing
            } try {
                hang.getCurrentCommand().cancel();
            }catch(NullPointerException e) {
                //nothing
            }
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        return m_AutoChooser.getSelected();
    }


    /**
     * Smooths joystic input for easier precice control without sacrificing full power.
     * @param in Input from joystic
     * @param deadzone Joystick deadzone
     * @return Transformed output
     */
    public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
    {
        return () -> {
            double x = in.get();
            if(Math.abs(x) < deadzone)
                return 0.0;
            else if (x>0)
                return Math.pow((x - deadzone)*(1.0/(1.0-deadzone)), 3);
            else 
                return -Math.pow((-x - deadzone)*(1.0/(1.0-deadzone)), 3);
        };
    }
}