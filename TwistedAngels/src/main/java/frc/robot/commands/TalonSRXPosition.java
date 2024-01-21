// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TalonSRXPosition extends Command{
    /** Creates a new TalonSRXPosition. */
    private TalonSRX motor;
    private double position;
    private int slotNumber;
    private double error;
    public TalonSRXPosition(TalonSRX m, double pos, int slot, double err, Subsystem s){
        // Use addRequirements() here to declare subsystem dependencies.
        motor = m;
        position = pos;
        slotNumber = slot;
        error = err;
        addRequirements(s);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize(){
        motor.selectProfileSlot(0, 0);
        motor.set(TalonSRXControlMode.Position, position);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){
        motor.selectProfileSlot(1, 0);
    }

    // Returns true when the command should end
    @Override
    public boolean isFinished(){
        return Math.abs(position - motor.getSelectedSensorPosition()) < error;
    }
}
