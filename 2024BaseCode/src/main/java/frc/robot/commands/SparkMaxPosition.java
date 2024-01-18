// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SparkMaxPosition extends Command{
    /** Creates a new SparkMaxPosition */
    private CANSparkMax motor;
    private double position;
    private int slotNumber;
    private double error;
    private Supplier<Double> feedback;

    public SparkMaxPosition(CANSparkMax m, double pos, int slot, double err, Subsystem s){
        // Use addRequirements() here to declare subsystem dependencies.
        motor = m;
        position = pos;
        slotNumber = slot;
        error = err;
        feedback = (motor.getEncoder())::getPosition;
        addRequirements(s);
    }

    public SparkMaxPosition(CANSparkMax m, double pos, int slot, double err, Subsystem s, Supplier<Double> feedback){
        this(m, pos, slot, err, s);
        this.feedback = feedback;
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        motor.getPIDController().setReference(position, ControlType.kPosition, slotNumber);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return Math.abs(position - feedback.get()) < error;
    }
}
