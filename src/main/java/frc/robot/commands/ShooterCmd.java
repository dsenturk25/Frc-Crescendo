
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final double speed;

    public ShooterCmd(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("ShooterCmd started!");
  }

  @Override
  public void execute() {
    shooterSubsystem.setMotor(speed);
    if(ShooterConstants.SHOOT_LOW_BUTTON){ //burasi olmadi
        shooterSubsystem.setMotor(ShooterConstants.LOW_SHOOT_SPEED);
        System.out.println("Robot is shooting higher");
    }
    else if(ShooterConstants.SHOOT_HIGH_BUTTON){ //burasi olmadi
        shooterSubsystem.setMotor(ShooterConstants.HIGH_SHOOT_SPEED);
        System.out.println("Robot is shooting Lower");
    }

  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMotor(0);
    System.out.println("ShooterCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}