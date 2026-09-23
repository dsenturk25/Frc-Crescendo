// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** An example command that uses an example subsystem. */
public class ClimberJoystickCmd extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final double speed;

    public ClimberJoystickCmd(ClimbSubsystem climbSubsystem, double speed){
        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ClimberJoystickCmd started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.setMotor(speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setMotor(0);
    System.out.println("ClimberJoystickCmd ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}