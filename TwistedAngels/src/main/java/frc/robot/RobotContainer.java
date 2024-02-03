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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
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

    //Auto Chooser
    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver::getLeftY, driver::getLeftX, driver::getRightX, true, driver::getR2Axis));
        aPivot = new APivot();
        intake = new Intake();
        hang = new Hang();
        shooter = new Shooter();
        sPivot = new SPivot();
        leds = new LEDs();
        shooterMode = new ShootingMode(driver, specialist);
        led1 = new AddressableLED(0);
        ledBuffer1 = new AddressableLEDBuffer(10);


        aPivot.setDefaultCommand(aPivot.joystickMove(specialist::getLeftY));
        sPivot.setDefaultCommand(sPivot.joystickControl(specialist::getRightY));

        // Configure the button bindings
        configureButtonBindings();

        // Add autonomous options to chooser
        m_AutoChooser.setDefaultOption("None", Autos.none());
        // TODO m_AutoChooser.addOption("PathPlanner Example", Autos.exampleAuto());
        m_AutoChooser.addOption("Test", Autos.testAuto());

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
            .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro(0))
            .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
        driver.cross().toggleOnTrue(shooterMode);
        specialist.R2().onTrue(new SequentialCommandGroup (
             intake.toShoot(),
             new WaitCommand(1.5),
             new InstantCommand( () -> shooter.getCurrentCommand().cancel()),
             sPivot.pivotBack()
             ).unless(shooter::readyShoot).withName("shoot"));
        specialist.L1().onTrue(intake.inAmp().unless(aPivot::isDown).withName("scoreInAmp"));

        specialist.povUp().onTrue(aPivot.pivotUp().withName("aPivotUp"));
        specialist.povDown().onTrue(aPivot.pivotDown().withName("aPivotDown"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        return m_AutoChooser.getSelected();
    }
}