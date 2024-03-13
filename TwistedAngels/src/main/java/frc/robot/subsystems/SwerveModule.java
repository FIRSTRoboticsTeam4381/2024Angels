package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.lib.math.Conversions;
import frc.lib.util.LogOrDash;
import frc.lib.util.SparkSaver;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveModule {
    public int moduleNumber;
    private CANSparkMax mAngleMotor;
    private CANSparkFlex mDriveMotor;

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

        //mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);

        // Inversion status seems to be lost if a drive wheel is inverted, so invert manually
        mDriveMotor.setInverted(false);
        

        /* Angle Encoder Config */
        absoluteEncoder = mAngleMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
        //absoluteEncoder.setPositionConversionFactor(360); Now set in config and saved to memory
        //absoluteEncoder.setZeroOffset(-180);

        //mAngleMotor.getPIDController().setFeedbackDevice(absoluteEncoder);
        //mAngleMotor.getPIDController().setPositionPIDWrappingMinInput(-180);
        //mAngleMotor.getPIDController().setPositionPIDWrappingMaxInput(180);
        //mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(true);

        distanceEncoder = mDriveMotor.getEncoder();
        // Set to m/s for speed and m for distance
        //distanceEncoder.setPositionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio);
        //distanceEncoder.setVelocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0);

        lastAngle = getState().angle.getDegrees();


        SparkSaver.optimizeCANFrames(mAngleMotor, false, false, false, false, false, true);
        SparkSaver.optimizeCANFrames(mDriveMotor, false, true, true, false, false, false);

        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); //TODO does this need to be update for Rev?

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(driveInvert() * percentOutput);
        }
        else{
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio); //TODO update for neos?
            mDriveMotor.getPIDController().setReference(driveInvert() * velocity, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond * driveInvert()));
        }

        //double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less than 1%. Prevents jittering.
        double angle = desiredState.angle.getDegrees();
        mAngleMotor.getPIDController().setReference(angle+180, ControlType.kPosition);
        desiredAngle = angle;
        lastAngle = angle;
    }
    
    /**
     * Set settings for this motor controller and save tham to its flash memory.
     * 
     * This is only intended to be done when hardware is replaced or settings changed,
     * NOT on each boot! This prevents failed configuration or carryover from previous code.
     */
    public Command configToFlash(Swerve s)
    {

        return new SparkSaver(mAngleMotor, "Angle"+moduleNumber, s)
            .configurePID(0, Constants.Swerve.angleKP,
                Constants.Swerve.angleKI,
                Constants.Swerve.angleKD,
                Constants.Swerve.angleKF)
            .setSmartCurrentLimit(Constants.Swerve.angleCurrentLimit)
            .setCoastMode() //Not sure why our base code was set to this
            .configurePIDSensor(absoluteEncoder)
            .configurePIDWrapping(0, 0, 360)
            .setSetting(() -> absoluteEncoder.setPositionConversionFactor(360), "Absolute Angle Scale")
            .setSetting(() -> absoluteEncoder.setZeroOffset(absoluteEncoder.getZeroOffset()), "Keep Absolute Angle Offset")
            .buildCommand()
            .andThen(new SparkSaver(mDriveMotor, "Drive"+moduleNumber, s)
            .configurePID(0, Constants.Swerve.driveKP,
                Constants.Swerve.driveKI,
                Constants.Swerve.driveKD,
                Constants.Swerve.driveKF)
            .setSmartCurrentLimit(Constants.Swerve.driveCurrentLimit)
            .setSecondaryCurrentLimit(70)
            .setBrakeMode()
            .setSetting(() -> 
                distanceEncoder.setPositionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio)
                , "Position Conversion Factor")
            .setSetting(() ->  
                distanceEncoder.setVelocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0)
                , "Velocity Conversion Factor")
            .buildCommand());
       
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition()-180);
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
        double velocity = distanceEncoder.getVelocity() * driveInvert(); //Units configured to m/s
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double distance = distanceEncoder.getPosition() * driveInvert(); //Units configured to m
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distance, angle);
    }

    public void sendTelemetry(){
        //LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getAngle().getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/position", getState().angle.getDegrees());
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/position", getPosition().distanceMeters);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/setpoint", desiredAngle);
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/setpoint", lastSpeed);
        
        LogOrDash.sparkDiagnostics("swerve/m" + moduleNumber + "/angle", mAngleMotor);
        LogOrDash.sparkDiagnostics("swerve/m" + moduleNumber + "/drive", mDriveMotor);

        LogOrDash.logNumber("swerve/m"+moduleNumber+"/angle/raw_analog", absoluteEncoder.getPosition());
    }


    // Stuff for SysID drivetrain tests
    public void setVoltage(double v)
    {
        mAngleMotor.getPIDController().setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(v);
    }

    public void logSysIDData(SysIdRoutineLog log)
    {
        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveMotor.getEncoder().getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));
    }

    public int driveInvert()
    {
        if(Constants.Swerve.driveMotorInvert)
            return -1;
        else
            return 1;
    }
}
