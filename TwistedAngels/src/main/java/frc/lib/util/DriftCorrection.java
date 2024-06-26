package frc.lib.util;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.util.LogOrDash;

public class DriftCorrection {
    
    /**
     * Drift correction PID used to correct our rotation in teleop
     * @param speeds Chassis speeds object from drive subsystem
     * @param pose Current pose from odometry
     * @return Updated Chassis Speeds
     */

    private static double lockAngle = 0;
    private static boolean locked = false;
    private static PIDController rotationCorrection = new PIDController(2.5, 0, 0);
    public static boolean enabled = true;
    
    private static double rawGyro = 0;

    public static void configPID()
    {
        rotationCorrection.enableContinuousInput(-Math.PI, Math.PI); //0-360
    }

    public static ChassisSpeeds driftCorrection(ChassisSpeeds speeds, Pose2d pose, AHRS gyro)
    {
        if(enabled)
        {

            LogOrDash.logBoolean("Rotation Locked", locked);
            LogOrDash.logNumber("Lock Angle", lockAngle);
            LogOrDash.logNumber("Current Angle", pose.getRotation().getRadians());

            SmartDashboard.putNumber("Rotation Natural Target", speeds.omegaRadiansPerSecond);

            if(speeds.omegaRadiansPerSecond == 0.0)
            {
                // Not trying to rotate, attempt to maintain angle
                rawGyro = gyro.getRate();
                if(locked)
                {
                    // Angle already locked on
                    speeds.omegaRadiansPerSecond = rotationCorrection.calculate(pose.getRotation().getRadians()); //PathPlanner uses radians
                    LogOrDash.logNumber("Rotation Correction", speeds.omegaRadiansPerSecond);
                    return speeds;
                }
                else if(Math.abs(rawGyro) < 0.1) // May have to tweak later
                {
                    // No angle locked, acquire lock
                    lockAngle = pose.getRotation().getRadians();// % 2*Math.PI;//% 360;
                    rotationCorrection.setSetpoint(lockAngle);
                    locked = true;
                    return speeds;
                }
                else
                {
                    return speeds;
                }
            }
            else
            {
                locked = false;
                return speeds;
            }
        }
        else
        {
            return speeds;
        }
    }

    /*public static ChassisSpeeds driftCorrection(ChassisSpeeds speeds, Pose2d pose) {
        PIDController driftCorrectionPID = new PIDController(0.07, 0.00, 0.004);
        double desiredHeading= 0;
        double pXY = 0;
        ChassisSpeeds newSpeeds = speeds;

        double xy = Math.abs(newSpeeds.vxMetersPerSecond) + Math.abs(newSpeeds.vyMetersPerSecond);

        if (Math.abs(newSpeeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0)
            desiredHeading = pose.getRotation().getDegrees();
        else if (xy > 0)
            newSpeeds.omegaRadiansPerSecond += driftCorrectionPID.calculate(pose.getRotation().getDegrees(),
                    desiredHeading);
        
        pXY = xy;

        return newSpeeds;
    }*/

    public static InstantCommand ampPoint(){
        return new InstantCommand(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if(alliance.isPresent()){
                if(alliance.get() == Alliance.Blue){
                    lockAngle = Math.PI/2.0;
                    rotationCorrection.setSetpoint(lockAngle);
                }else{
                    lockAngle = -Math.PI/2.0;
                    rotationCorrection.setSetpoint(lockAngle);
                }
            }
            
        });
    }

}
