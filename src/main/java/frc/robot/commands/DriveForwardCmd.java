
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveForwardCmd extends Command {

    private DriveSubsystem driveSubsystem;
    private double speed;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double speed){
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
        addRequirements();
    }

  @Override
  public void initialize() {
    System.out.println("DriveForwardCmd started!");
  }

  @Override
  public void execute() {
    driveSubsystem.driveMotors(0, speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveMotors(0, 0, 0);
    System.out.println("DriveForwardCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}