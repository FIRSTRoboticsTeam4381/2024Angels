// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** Helper class for safely saving settings to Spark motor controllers flash memory. */
public class SparkSaver {

    private ArrayList<SettingToSave> settings;
    
    private CANSparkBase controller;
    private String name;
    private Subsystem subsystem;

    /**
     * Create a SparkSaver object for saving the settings on a single motor controller.
     * @param controller The motor controller to configure
     * @param name A user-friendly name identifying which motor controller this is, to help with diagnosing issues
     * @param subsystem The subsystem this motor controller belongs to, so that it can be made busy when saving
     */
    public SparkSaver(CANSparkBase controller, String name, Subsystem subsystem)
    {
        this.controller = controller;
        this.name = name;
        this.subsystem = subsystem;

        settings = new ArrayList<SettingToSave>();
    }

    private SparkSaver()
    {
        // Do not use!
    }


    // TODO add commonly used settings as premade methods

    /**
     * Sets smart current limit. See documentation on motor controller method for details.
     * @param limit Current limit in amps
     * @return the SparkSaver to allow chaining settings
     */
    public SparkSaver setSmartCurrentLimit(int limit)
    {
        setSetting(() -> controller.setSmartCurrentLimit(limit), "Current Limit");
        return this;
    }

    /**
     * Sets smart current limit. See documentation on motor controller method for details.
     * @param stalllLimit Current limit in amps at stall
     * @param freeLimit Current limit at free speed
     * @param limitRPM RPM at which to transition between current limits (see rev docs)
     * @return the SparkSaver to allow chaining settings
     */
    public SparkSaver setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM)
    {
        setSetting(() -> controller.setSmartCurrentLimit(stallLimit,freeLimit,limitRPM), "Current Limit");
        return this;
    }

    /**
     * Sets motor controller to brake mode when zero output is applied.
     * @return the SparkSaver to allow chaining settings
     */
    public SparkSaver setBrakeMode()
    {
        setSetting(() -> controller.setIdleMode(IdleMode.kBrake), "Brake Mode");
        return this;
    }

    /**
     * Sets motor controller to coast mode when zero output is applied.
     * @return the SparkSaver to allow chaining settings
     */
    public SparkSaver setCoastMode()
    {
        setSetting(() -> controller.setIdleMode(IdleMode.kCoast), "Coast Mode");
        return this;
    }

    public SparkSaver setOpenLoopRampRate(double rate)
    {
        setSetting(() -> controller.setOpenLoopRampRate(rate), "Open Loop Ramp Rate");
        return this;
    }

    public SparkSaver setClosedLoopRampRate(double rate)
    {
        setSetting(() -> controller.setClosedLoopRampRate(rate), "Open Loop Ramp Rate");
        return this;
    }

    public SparkSaver setSoftLimits(int lowerLimit, int upperLimit)
    {
        setSetting(() -> controller.setSoftLimit(SoftLimitDirection.kReverse, upperLimit), "Lower Limit");
        setSetting(() -> controller.setSoftLimit(SoftLimitDirection.kForward, upperLimit), "Upper Limit");
        return this;
    }
    

    public SparkSaver follow(CANSparkBase toFollow, boolean invert)
    {
        setSetting(() -> controller.follow(toFollow, invert), "Follow");
        return this;
    }

    public SparkSaver configurePIDSensor(MotorFeedbackSensor sensorToUse)
    {
        setSetting(() -> controller.getPIDController().setFeedbackDevice(sensorToUse), "PID Feedback Device");
        return this;
    }

    public SparkSaver configurePID(int slot, double p, double i, double d, double f)
    {
        setSetting(() -> controller.getPIDController().setP(p, slot), "PID P");
        setSetting(() -> controller.getPIDController().setI(i, slot), "PID I");
        setSetting(() -> controller.getPIDController().setD(d, slot), "PID D");
        setSetting(() -> controller.getPIDController().setFF(f, slot), "PID F");

        return this;
    }

    public SparkSaver configureDFilter(int slot, double gain)
    {
        setSetting(() -> controller.getPIDController().setDFilter(gain, slot), "D Filer");

        return this;
    }

    public SparkSaver configureIZone(int slot, double zone)
    {
        setSetting(() -> controller.getPIDController().setIZone(zone, slot), "I Zone");

        return this;
    }

    public SparkSaver configureIMax(int slot, double max)
    {
        setSetting(() -> controller.getPIDController().setIMaxAccum(max, slot), "I Max");

        return this;
    }

    public SparkSaver configurePIDWrapping(int slot, double min, double max)
    {
        setSetting(() -> controller.getPIDController().setPositionPIDWrappingEnabled(true), "PID Wrap Enable");
        setSetting(() -> controller.getPIDController().setPositionPIDWrappingMinInput(min), "PID Wrap Min");
        setSetting(() -> controller.getPIDController().setPositionPIDWrappingMinInput(max), "PID Wrap Max");
        return this;
    }

    public SparkSaver configurePIDOutputRange(int slot, double min, double max)
    {
        setSetting(() -> controller.getPIDController().setOutputRange(min, max), "PID Output Range");
        return this;
    }
    


    /**
     * Add a custom setting to be saved, which returns a REVLibError on save.
     * Tip: If a setting doesn't return a REVLibError, it is probably set locally
     * (i.e. on the RoboRIO) and can't actually be saved to the Spark's memory.
     * @param setting Supplier for the setting to save
     * @param name Name of setting being saved, to help diagnose if setting isn't set
     */
    public SparkSaver setSetting(Supplier<REVLibError> setting, String name)
    {
        settings.add(new SettingToSave(setting, name));
        return this;
    }

    /**
     * Create a command that will safely save the settings added so far to the controller.
     * IMPORTANT: This is meant to be added to a dashboard button or similar for saving settings.
     * Settings should not be saved every run, and certainly not while the robot is enabled!
     * @return Command that, when run, will save settings to the motor controller.
     */
    public Command buildCommand()
    {
        return new InstantCommand(() -> {

            controller.setCANTimeout(1000);
            
            try
            {
                DriverStation.reportWarning("Coniguring "+name+" on "+subsystem.getName()+", please do not turn off the robot!", false);

                // Reset to default
                trySetting(new SettingToSave(controller::restoreFactoryDefaults, "Initial Reset To Default"));

                Thread.sleep(1000);

                for(SettingToSave s : settings)
                {
                    // Abort if failed
                    if(trySetting(s))
                        return;
                }

                // Save settings to controller memory
                Thread.sleep(1000);
                trySetting(new SettingToSave(controller::burnFlash, "Final Save To Flash"));;
                Thread.sleep(1000);


                DriverStation.reportWarning("Successfully configured "+name+" on "+subsystem.getName()+"!\n"+
                "WARNING: Reboot the robot before running this motor, as unsavable settings have been lost!", false);
            }
            catch(InterruptedException e)
            {
                DriverStation.reportError("Thread interrupted while configuring "+name+" on "+subsystem.getName()+
                ", aborting!\n"+"Do NOT attempt to run the motor above until robot has rebooted!", false);
            }

            controller.setCANTimeout(0);

        }).ignoringDisable(true);
    }

    private boolean trySetting(SettingToSave s)
    {
        REVLibError result = s.setting.get();
        if(result == REVLibError.kOk)
        {
            return false;
        }
        else
        {
            // Something went wrong
            DriverStation.reportError(
                "Error while configuring "+name+" of subsystem "+subsystem.getName()+":\n"
                +s.label+" returned error "+result.name(), true);
            DriverStation.reportError("Aborting settings save, please try again!\n"
                +"Do NOT attempt to run the motor above until robot has rebooted!", false);
            return true;
        }
    }


    /**
     * Helper class bundling a setting to be saved and a label for the setting.
     */
    private class SettingToSave
    {
        public Supplier<REVLibError> setting;
        public String label;

        public SettingToSave(Supplier<REVLibError> setting, String label)
        {
            this.setting = setting;
            this.label = label;
        }
    }
}
