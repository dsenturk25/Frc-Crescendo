
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbCmd extends Command {

    private final ClimbSubsystem climbSubsystem;
    private final double speed;

    public ClimbCmd(ClimbSubsystem climbSubsystem, double speed){
        this.climbSubsystem = climbSubsystem;
        this.speed = speed;
        addRequirements(climbSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("ClimberJoystickCmd started!");
  }

  @Override
  public void execute() {
    climbSubsystem.setMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setMotor(0);
    System.out.println("ClimberJoystickCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}