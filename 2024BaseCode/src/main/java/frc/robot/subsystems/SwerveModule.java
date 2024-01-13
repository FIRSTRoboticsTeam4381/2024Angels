package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder.Type;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.LogOrDash;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class SwerveModule {
    public int moduleNumber;
    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;

    private SparkAbsoluteEncoder absoluteEncoder;

    private RelativeEncoder distanceEncoder;

    private double lastAngle;
    private double desiredAngle;
    private double lastSpeed;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        //configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        //configDriveMotor();

        /* Angle Encoder Config */
        absoluteEncoder = mAngleMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360);
        absoluteEncoder.setZeroOffset(-180);

        mAngleMotor.getPIDController().setFeedbackDevice(absoluteEncoder);
        mAngleMotor.getPIDController().setPositionPIDWrappingMinInput(-180);
        mAngleMotor.getPIDController().setPositionPIDWrappingMaxInput(180);
        mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(true);

        distanceEncoder = mDriveMotor.getEncoder(Type.kHallSensor, Constants.NEO_TICKS_PER_REV);
        // Set to m/s for speed and m for distance
        distanceEncoder.setPositionConversionFactor(Constants.Swerve.wheelDiameter / Constants.Swerve.driveGearRatio);
        distanceEncoder.setVelocityConversionFactor(Constants.Swerve.wheelDiameter / Constants.Swerve.driveGearRatio / 60.0);

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //TODO does this need to be update for Rev?

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else{
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio); //TODO update for neos?
            mDriveMotor.getPIDController().setReference(velocity, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less than 1%. Prevents jittering.
        mAngleMotor.getPIDController().setReference(angle, ControlType.kPosition);
        desiredAngle = angle;
        lastAngle = angle;
    }
    
    /**
     * Set settings for this motor controller and save tham to its flash memory.
     * 
     * This is only intended to be done when hardware is replace or settings changed,
     * NOT on each boot! This prevents failed configuration or carryover from previous code.
     */
    public void configToFlash()
    {
        try
        {
            // Drive motor
            LogOrDash.checkRevError("drive motor "+moduleNumber+" clear",
                mDriveMotor.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            SparkPIDController pid = mDriveMotor.getPIDController();

            LogOrDash.checkRevError("drive motor "+moduleNumber+" kp",
                pid.setP(Constants.Swerve.driveKP));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" ki",
                pid.setI(Constants.Swerve.driveKI));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kd",
                pid.setD(Constants.Swerve.driveKD));
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kf",
                pid.setFF(Constants.Swerve.driveKF));
                
            LogOrDash.checkRevError("drive motor "+moduleNumber+" open loop ramp",
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp));

            LogOrDash.checkRevError("drive motor "+moduleNumber+" closed loop ramp",
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.closedLoopRamp));
            
            LogOrDash.checkRevError("drive motor "+moduleNumber+" current",
                mDriveMotor.setSmartCurrentLimit(Constants.Swerve.drivePeakCurrentLimit, Constants.Swerve.driveContinuousCurrentLimit));

            LogOrDash.checkRevError("drive motor "+moduleNumber+" idle mode",
                mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode));

            // This doesn't return a RevLibError apparently
            mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);

            Thread.sleep(1000);
            LogOrDash.checkRevError("drive motor "+moduleNumber+" BURN",
                mDriveMotor.burnFlash());
            Thread.sleep(1000);

            // Anale motor
            LogOrDash.checkRevError("angle motor "+moduleNumber+" clear",
                mAngleMotor.restoreFactoryDefaults());
            
            Thread.sleep(1000);

            pid = mAngleMotor.getPIDController();

            LogOrDash.checkRevError("angle motor "+moduleNumber+" kp",
                pid.setP(Constants.Swerve.angleKP));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" ki",
                pid.setI(Constants.Swerve.angleKI));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kd",
                pid.setD(Constants.Swerve.angleKD));
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kf",
                pid.setFF(Constants.Swerve.angleKF));
            
            LogOrDash.checkRevError("angle motor "+moduleNumber+" current",
                mAngleMotor.setSmartCurrentLimit(Constants.Swerve.anglePeakCurrentLimit, Constants.Swerve.angleContinuousCurrentLimit));

            LogOrDash.checkRevError("angle motor "+moduleNumber+" idle mode",
                mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode));

            // This doesn't return a RevLibError apparently
            mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);

            Thread.sleep(1000);
            LogOrDash.checkRevError("angle motor "+moduleNumber+" BURN",
                mAngleMotor.burnFlash());
            Thread.sleep(1000);
        }
        catch(InterruptedException e)
        {
            DriverStation.reportError("Main thread interrupted while flashing swerve module!", e.getStackTrace());
        }
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition());
    }

    /**
     * Get temp of a motor in this swerve module
     * @param motor motor index 1 is drive motor, any other number is angle motor
     * @return
     */
    public Double getTemp(int motor){
        return (motor == 1)?mDriveMotor.getMotorTemperature():mAngleMotor.getMotorTemperature();
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public double getDesiredSpeed(){
        return lastSpeed;
    }

    public SwerveModuleState getState(){
        double velocity = distanceEncoder.getVelocity(); //Units configured to m/s
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double distance = distanceEncoder.getPosition(); //Units configured to m
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distance, angle);
    }

    public void sendTelemetry(){
        //LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getAngle().getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/position", getState().angle.getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getPosition().distanceMeters);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/setpoint", desiredAngle);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/setpoint", lastSpeed);
        
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/angle", mAngleMotor);
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/drive", mDriveMotor);

        LogOrDash.logNumber("swerve/m"+moduleNumber+"/angle/raw_analog", absoluteEncoder.getPosition());
    }
}
