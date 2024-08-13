// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.DriftCorrection;
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
import frc.robot.subsystems.NoteTracker;
import frc.robot.subsystems.PhotonCam;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aimbot;
import frc.robot.commands.NoteLineup;
//import frc.robot.commands.Ampbot;
import frc.robot.commands.ShootingMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in 2the {@link Robot}
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
    public static Command shooterMode;
    public static Command aimbot;
    public static Command ampbot;
    public static AddressableLED led1;
    public static AddressableLEDBuffer ledBuffer1;
    public static Limelight limelight;
    public static PhotonCam camA; 
     public static PhotonCam camB;
     public static NoteTracker nt;

    //Auto Chooser
    SendableChooser<Autos.PreviewAuto> m_AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, 
            driver::getLeftY,
            driver::getLeftX,
          //interpolateJoystick(driver::getLeftY,0.05),
          //interpolateJoystick(driver::getLeftX,0.05), 
          interpolateJoystick (driver::getRightX,0.05),
             true, driver.R1()::getAsBoolean));
        aPivot = new APivot();
        intake = new Intake();
        hang = new Hang();
        shooter = new Shooter();
        sPivot = new SPivot();
        camA = new PhotonCam("Camera A", new Transform3d(new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(-12.5),  Units.inchesToMeters(15)), new Rotation3d(0,Math.PI/-6,Math.PI/-4-Math.PI)) );
        camB = new PhotonCam("Camera B", new Transform3d(new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(12.5),  Units.inchesToMeters(15)), new Rotation3d(0,Math.PI/-6,Math.PI/4-Math.PI)) );
        nt = new NoteTracker("NoteTracker"); 
        leds = new LEDs();
        limelight = new Limelight();
        shooterMode = new ShootingMode(driver, specialist);
        aimbot = new Aimbot(interpolateJoystick(driver::getLeftY,0.05),
            interpolateJoystick(driver::getLeftX,0.05), driver.R1()::getAsBoolean);//.until(driver.cross()::getAsBoolean).withName("Aimbot");
        //ampbot = new Ampbot(interpolateJoystick(driver::getLeftY,0.05),
        //    interpolateJoystick(driver::getLeftX,0.05), driver::getR2Axis);
        //led1 = new AddressableLED(2);
        ledBuffer1 = new AddressableLEDBuffer(10);
        
        aPivot.setDefaultCommand(aPivot.joystickMove(interpolateJoystick(specialist::getLeftY, 0.05)));
        sPivot.setDefaultCommand(sPivot.joystickControl(interpolateJoystick(specialist::getRightY, 0.05)));
        hang.setDefaultCommand(hang.hangTriggers(specialist::getR2Axis, specialist::getL2Axis));

        // Configure the button bindings
        configureButtonBindings();

        // Add autonomous options to chooser
        m_AutoChooser.setDefaultOption("None", Autos.none());
        // TODO m_AutoChooser.addOption("PathPlanner Example", Autos.exampleAuto());
        //m_AutoChooser.addOption("Test", Autos.testAuto());
        m_AutoChooser.addOption("3NoteFront", Autos.Front3Note());
        //m_AutoChooser.addOption("3NoteFrontRED", Autos.Front3NoteRED());
        //m_AutoChooser.addOption("3NoteFront2", Autos.Front3Note2());
        m_AutoChooser.addOption("DefenseInAuto", Autos.DefenseInAuto());
        m_AutoChooser.addOption("MiddleNotesCenter", Autos.middleNotesCenter());
        m_AutoChooser.addOption("MiddleNotesSource", Autos.middleNotesSource());
        //m_AutoChooser.addOption("MiddleNotesSourceRED", Autos.middleNotesSRED());
        //m_AutoChooser.addOption("MiddleNotesCenterRED", Autos.middleNotesCRED());
        m_AutoChooser.addOption("ShootCenter", Autos.RedShootCenter());
        m_AutoChooser.addOption("ShootSource", Autos.RedShootSource());
        //m_AutoChooser.addOption("Ampside3NoteBLUE", Autos.Ampside3NoteBLUE());
        //m_AutoChooser.addOption("Ampside3NoteRED", Autos.Ampside3NoteRED());
        //m_AutoChooser.addOption("Middle2Note", Autos.Middle2Note());


        SmartDashboard.putData("Choose Auto:", m_AutoChooser);

        SmartDashboard.putData(CommandScheduler.getInstance());

        SmartDashboard.putString("Choose Notes", "");

        m_AutoChooser.onChange((listener) -> listener.showPreview());

        SmartDashboard.putNumber("Start Delay",0);

        DriftCorrection.configPID();
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
            .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.5, 7.75, Rotation2d.fromDegrees(0))))));
        
        // Toggle shoot mode on/off, switching to aimbot when specials triangle is held
        // This is one of those things that is way too complicated in command-based code          
        driver.cross().onTrue(new ConditionalCommand(
            new InstantCommand(() -> CommandScheduler.getInstance().cancel(shooterMode, aimbot)), 
            new ScheduleCommand(shooterMode),
            () -> shooter.getCurrentCommand() != null));

        // Toggle to auto aim
        specialist.triangle().and(shooterMode::isScheduled).onTrue(aimbot);
        // Can't write this like previous line or driver x button also triggers this line, causing a loop of schedule-cancel
        specialist.triangle().onFalse(new ProxyCommand(shooterMode).onlyIf(aimbot::isScheduled));

        // Shoot if ready
        specialist.R1().onTrue(new SequentialCommandGroup (
             intake.toShoot(),
             new WaitCommand(.7),
             new InstantCommand( () -> {
                Command c = shooter.getCurrentCommand();
                if(c != null)
                    c.cancel();
            }),
             new ProxyCommand(sPivot.pivotBack())
             ).onlyIf(shooter::readyShoot).withName("shoot"));

        // Score in amp if up
        specialist.L1().onTrue(intake.inAmp().unless(aPivot::isDown).withName("scoreInAmp"));

        // Amp pivot snap to position
        specialist.povUp().onTrue(aPivot.pivotUp().withName("aPivotUp"));
        specialist.povDown().onTrue(aPivot.pivotDown().withName("aPivotDown"));

        // Intake & eject
        specialist.circle().toggleOnTrue(intake.pickup().withName("pickup"));
        specialist.cross().onTrue(intake.spitOut().withName("spitOut"));

        // Cancel ongoing commands
        specialist.PS().onTrue(new InstantCommand(() -> { // Cancel all commands
            CommandScheduler.getInstance().cancelAll();
        }));

        // Shooter Positions
        specialist.povRight().onTrue(sPivot.pivotToCloseShot());

        driver.L1().whileTrue(s_Swerve.setCoast());
        driver.square().onTrue(DriftCorrection.ampPoint());

        //
        specialist.square().onTrue(new ParallelRaceGroup(
            shooter.pass(),
            new SequentialCommandGroup(
                sPivot.pass(), 
                new RunCommand(()-> {}).until(shooter::readyPass),
                intake.toShoot()//,
                 //sPivot.pivotBack()
            )

        ));
    driver.triangle().whileTrue(new NoteLineup(s_Swerve, nt,false));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        Autos.pickNotes();
        double startDelay=SmartDashboard.getNumber("Start Delay", 0);
         return new SequentialCommandGroup( 
            new WaitCommand(startDelay), 
            new ScheduleCommand(m_AutoChooser.getSelected().auto)); 
    
    }


    /**
     * Smooths joystic input for easier precice control without sacrificing full power.
     * @param in Input from joystic
     * @param deadzone Joystick deadzone
     * @return Transformed output
     */
    public static Supplier<Double> interpolateJoystick(Supplier<Double> in, double deadzone)
    {
        return () -> interpolateNow(in.get(), deadzone);
    }

    public static double interpolateNow(double in, double deadzone)
    {
        if(Math.abs(in) < deadzone)
            return 0.0;
        else if (in>0)
            return Math.pow((in - deadzone)*(1.0/(1.0-deadzone)), 3);
        else 
            return -Math.pow((-in - deadzone)*(1.0/(1.0-deadzone)), 3);
    }
}