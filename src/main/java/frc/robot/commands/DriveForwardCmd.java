
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveForwardCmd extends Command {

    private DriveSubsystem driveSubsystem;
    private double speed;
    private double startTimestamp;
    private double time;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double speed, double time){
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
        this.startTimestamp = Timer.getFPGATimestamp();
        this.time = time;
        addRequirements(driveSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("DriveForwardCmd started!");
  }

  @Override
  public void execute() {
    if ((Timer.getFPGATimestamp() - startTimestamp) < (time * 0.9) /* Will cut power to stop */) {
      driveSubsystem.driveMotors(speed, 0, 0);
    } else {
      end(true);
    }
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