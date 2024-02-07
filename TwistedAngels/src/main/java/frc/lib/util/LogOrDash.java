// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.HashMap;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/** Add your docs here. */
public class LogOrDash {
    private static DataLog l;

    private static HashMap<String, DoubleLogEntry> doubles;
    private static HashMap<String, BooleanLogEntry> booleans;
    private static HashMap<String, StringLogEntry> strings;

    private static boolean sendToDash;

    public static void setupLogging()
    {
        // Start data logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        l = DataLogManager.getLog();

        doubles = new HashMap<String, DoubleLogEntry>();
        booleans = new HashMap<String, BooleanLogEntry>();
        strings = new HashMap<String, StringLogEntry>();

        sendToDash = true;

        SmartDashboard.putBoolean("Sending Debug Data", sendToDash);
        SmartDashboard.putData("Toggle Debug Data", new InstantCommand(() -> {
            sendToDash = !sendToDash;
            SmartDashboard.putBoolean("Sending Debug Data", sendToDash);
            DriverStation.reportError("Toggling dashboard", sendToDash);
        }).ignoringDisable(true));
        
    }

    public static void logNumber(String key, double value)
    {
        if(sendToDash)
        {
            SmartDashboard.putNumber(key, value);
        }
        else
        {
            DoubleLogEntry d = doubles.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new DoubleLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                doubles.put(key, d);
            }
        }
    }

    
    public static void logBoolean(String key, boolean value)
    {
        if(sendToDash)
        {
            SmartDashboard.putBoolean(key, value);
        }
        else
        {
            BooleanLogEntry d = booleans.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new BooleanLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                booleans.put(key, d);
            }
        }
    }

    
    public static void logString(String key, String value)
    {
        if(sendToDash)
        {
            SmartDashboard.putString(key, value);
        }
        else
        {
            StringLogEntry d = strings.get(key);
            if(d != null)
            {
                d.append(value);
            }
            else
            {
                d = new StringLogEntry(l, "NT:/SmartDashboard/"+key);
                d.append(value);
                strings.put(key, d);
            }
        }
    }


    /**
     * Prints the status code generated by the given rev control command 
     * to the driver station. This is intended to be used when configuring motors, 
     * not on every frame when setting motor speeds!
     * 
     * @param name Name of this line to print (to track down what didn't work)
     * @param e Put your REVLib method call here
     */
    public static void checkRevError(String name, REVLibError e)
    {
        DriverStation.reportError(name+" result: "+
                e
                .name(), false);
    }


    /**
     * Log or send to dashboard diagnostic information about a SPARK MAX.
     * 
     * This does NOT include information from external sensors (encoders etc).
     * @param prefix Prefix string for this motor, ex: "arm/motor1"
     * @param s The CAN SPARK MAX to log diagnostics of
     */
    public static void sparkDiagnostics(String prefix, CANSparkBase s)
    {
        // Motor temperature
        // Brushed motors won't have a connected internal thermister
        if(s.getMotorType() == MotorType.kBrushless)
        {
            logNumber(prefix+"/motorTemp", s.getMotorTemperature());
        }

        // Voltage & current data
        logNumber(prefix+"/outputCurrent", s.getOutputCurrent());
        logNumber(prefix+"/appliedOutput", s.getAppliedOutput());
        logNumber(prefix+"/busVoltage", s.getBusVoltage());

        // Error status
        logString(prefix+"/errors", s.getLastError().name());

        // Record faults and sticky faults
        logString(prefix+"/faults", decodeSparkMaxFaults(s.getFaults()));
        logString(prefix+"/stickyfaults", decodeSparkMaxFaults(s.getStickyFaults()));
        
    }

    /**
     * Decode SPARK MAX faults and sticky faults from a short
     * @param faults
     * @return String of faults, or empty string if none
     */
    private static String decodeSparkMaxFaults(short faultBits)
    {
        String faults = "";

        // Walk down the binary string, removing bits and adding to error string as we find them
        // TODO Skip one bit for negative numbers (unused) or get unsigned short instead?
        for(int i=15; i > 0; i--)
        {
            int x = (int) Math.pow(2, i);
            if(faultBits >= x)
            {
                faultBits -= x;
                FaultID f = FaultID.fromId(x);
                if(f != null)
                {
                    faults += f.name() + ", ";
                }
                else
                {
                    faults += ", unknown bit: "+x;
                }
            }
        }

        return faults;
    }


    /**
     * Logs diagnostic information about the Power Distribution Panel, such as blown fuses.
     * This does not log channel current, overall current or voltage because those are automatically
     * logged by having the PDP on the SmartDashboard.
     * @param pdp The Power Distribution Panel to log
     */
    public static void logPDPData(PowerDistribution pdp) {
        // Some stats that aren't auto-logged
        logNumber("pdp/temperature", pdp.getTemperature());
        logNumber("pdp/watts", pdp.getTotalPower());
        logNumber("pdp/joules", pdp.getTotalEnergy());
        
        // General faults
        String f = "";
        PowerDistributionFaults faults = pdp.getFaults();
        if(faults.Brownout)
            f += "Brownout, ";
        if(faults.CanWarning)
            f += "CAN Error, ";
        if(faults.HardwareFault)
            f += "Hardware Failure, ";

        logString("pdp/faults", f);

        // Sticky faults
        f = "";
        PowerDistributionStickyFaults stickies = pdp.getStickyFaults();
        if(stickies.Brownout)
            f += "Brownout, ";
        if(stickies.CanWarning)
            f += "CAN Error, ";
        if(stickies.CanBusOff)
            f += "CAN Offline, ";
        if(stickies.HasReset)
            f += "Rebooted, ";

        logString("pdp/sticky", f);

        // Blown fuses
        f = "";
        String s = "";
        for(int j=0; j<pdp.getNumChannels(); j++)
        {
            if(faults.getBreakerFault(j))
                f += j+", ";

            if(stickies.getBreakerFault(j))
                s += j+", ";
        }

        logString("pdp/blownfuses", f);
        logString("pdp/stickyfuses", s);
    }


    public static void setupSysIDTests(CANSparkBase[] toSetVoltage, CANSparkBase[] toLog, Subsystem s)
    {
       setupSysIDTests(new SysIdRoutine.Config(), toSetVoltage, toLog, s);
    }

    public static void setupSysIDTests(SysIdRoutine.Config c, CANSparkBase[] toSetVoltage, CANSparkBase[] toLog, Subsystem s)
    {
         SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (v) -> {
                    for(int j = 0; j < toSetVoltage.length; j++)
                    {
                        toSetVoltage[j].setVoltage(v.in(Units.Volts));
                    }
                }, 
                (log) ->
                {
                    for(int j = 0; j < toLog.length; j++)
                    {
                        log.motor("m"+j).voltage(
                        Units.Volts.of(   toLog[j].getAppliedOutput() * RobotController.getBatteryVoltage())
                        ).linearVelocity(Units.MetersPerSecond.of(toLog[j].getEncoder().getVelocity()))
                        .linearPosition(Units.Meters.of(toLog[j].getEncoder().getPosition()));
                    }
                }, 
                s)
        );

        SmartDashboard.putData("SysID/"+s.getName()+"/dyn_f", routine.dynamic(Direction.kForward));
        SmartDashboard.putData("SysID/"+s.getName()+"/dyn_r", routine.dynamic(Direction.kReverse));
        SmartDashboard.putData("SysID/"+s.getName()+"/quas_f", routine.dynamic(Direction.kForward));
        SmartDashboard.putData("SysID/"+s.getName()+"/quas_r", routine.dynamic(Direction.kReverse));
    }
}
