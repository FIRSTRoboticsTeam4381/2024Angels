package frc.robot.subsystems;

import java.io.ObjectInputStream.GetField;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.DriftCorrection;
import frc.lib.util.LogOrDash;
import frc.robot.Constants;

public class Swerve extends SubsystemBase{
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro; // Changed from Pigeon2

    public final Field2d field = new Field2d();
    private Pose2d startPose = new Pose2d(Units.inchesToMeters(177), Units.inchesToMeters(214), Rotation2d.fromDegrees(0));


    public Swerve(){
        gyro = new AHRS(SerialPort.Port.kUSB); // Changed from Pigeon2
        zeroGyro();

        SmartDashboard.putData(this);

        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), startPose);

        // TODO check - auto
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.Swerve.holonomicConfig,
            () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if(alliance.isPresent()) {
                        return alliance.get() == Alliance.Red;
                    }
                    return false;
                },
            this // Reference to this subsystem to set requirements
        );

        // Setup position logging
        SmartDashboard.putData("Field", field);
        //m_field.setRobotPose(startPose);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

          // Button to turn on/off sending debug data to the dashboard
          SmartDashboard.putData("Configure Swerve", configToFlash());


          setupSysIDTests();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.discretize(DriftCorrection.driftCorrection(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw()
                )
                : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation),
                swerveOdometry.getEstimatedPosition(),
                gyro),0.02
                ));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // TODO check - auto
    public void drive(ChassisSpeeds robotRelativeSpeeds){
        Translation2d translation = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
        double rotation = robotRelativeSpeeds.omegaRadiansPerSecond;
        drive(translation, rotation, false, false);
    }

    // TODO check - auto
    /* Used by PathPlanner AutoBuilder */
    private ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Command brake() {
        return new RunCommand(() -> {
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        },this);
    }

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose(){
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose){
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw){
        swerveOdometry.resetPosition(yaw, getPositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return Swerve Module positions
     */
    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Use to reset angle to certain known angle or to zero
     * @param angle Desired new angle
     */
    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw(){
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getPositions());

        // Call here
        
        LogOrDash.logNumber("Gyro Angle", getYaw().getDegrees());

        SwerveModuleState[] currentStatus = new SwerveModuleState[4];
        double[] targetSpeeds = new double[4];
        double[] targetAngles = new double[4];
        double[] absoluteAngles = new double[4];
        
        for(SwerveModule mod : mSwerveMods){
            mod.sendTelemetry();
            currentStatus[mod.moduleNumber] = mod.getState();
            targetSpeeds[mod.moduleNumber] = mod.getDesiredSpeed();
            targetAngles[mod.moduleNumber] = mod.getDesiredAngle();
            absoluteAngles[mod.moduleNumber] = mod.getAngle().getDegrees();
        }

        // Compile swerve status for AdvantageScope
        double[] targetStateAdv = new double[8];
        double[] currentStateAdv = new double[8];
        double[] absoluteStateAdv = new double[8];
        for(int i=0; i<4;i++)
        {
            targetStateAdv[2*i] = targetAngles[i];
            targetStateAdv[2*i+1] = targetSpeeds[i];
            
            currentStateAdv[2*i] = currentStatus[i].angle.getDegrees();
            currentStateAdv[2*i+1] = currentStatus[i].speedMetersPerSecond;

            absoluteStateAdv[2*i] = absoluteAngles[i];
            absoluteStateAdv[2*i+1] = 8;//Arbitrary to make these easier to see
        }

        SmartDashboard.putNumberArray("swerve/status", currentStateAdv);
        SmartDashboard.putNumberArray("swerve/target", targetStateAdv);
        SmartDashboard.putNumberArray("swerve/absolute", absoluteStateAdv);


        LogOrDash.logNumber("Gyro Pitch", gyro.getPitch());
        LogOrDash.logNumber("Gyro Roll", gyro.getRoll());
        LogOrDash.logNumber("Gyro Yaw", gyro.getYaw());
        
        LogOrDash.logNumber("Gyro Yaw Velocity", gyro.getRate());

        field.setRobotPose(getPose());
    }

    public Command configToFlash(){
        return new SequentialCommandGroup(
            mSwerveMods[0].configToFlash(this),
            mSwerveMods[1].configToFlash(this),
            mSwerveMods[2].configToFlash(this),
            mSwerveMods[3].configToFlash(this) //Probably a nicer way to iterate through these
        ).ignoringDisable(true);
    }


    private void setupSysIDTests() {

        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (v) -> {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].setVoltage(v.in(edu.wpi.first.units.Units.Volts));
                        }
                    }, 
                    (log) ->
                    {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].logSysIDData(log);
                        }
                    }, 
                    this)
            );

            SmartDashboard.putData("SysID/drive/dyn_f", routine.dynamic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/dyn_r", routine.dynamic(Direction.kReverse));
            SmartDashboard.putData("SysID/drive/quas_f", routine.quasistatic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/quas_r", routine.quasistatic(Direction.kReverse));

    }

    public Command setCoast() {
        return new StartEndCommand(() -> {
            for(SwerveModule mod : mSwerveMods){
                mod.setDriveIdleMode(IdleMode.kCoast);
        }
        },() -> {for(SwerveModule mod : mSwerveMods){
                mod.setDriveIdleMode(IdleMode.kBrake);
        }
    }).ignoringDisable(true);         
    }
    
    public void ResetToEdge() {
        if (getPose().getY() > 8.31) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(getPose().getX(), 8.31, getYaw()));        // Need to replace getPose(), getPost gets the current position, we need the desired position in the field
          } else if (getPose().getY() < -0.1) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(getPose().getX(), -0.1, getYaw()));      
          } if (getPose().getX() > 16.6) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(16.6, getPose().getY(), getYaw())); 
          } else if (getPose().getX() < -0.1) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(-0.1, getPose().getY(), getYaw())); 
          }
    }
}
