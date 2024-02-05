// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MechanumDriveCmd extends Command {

  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> xSpeedFunction, ySpeedFunction, rotationFunction;

  public MechanumDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> rotationFunction) {

    this.driveSubsystem = driveSubsystem;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunction = ySpeedFunction;
    this.rotationFunction = rotationFunction;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetGyro();
    System.out.println("Mechanum Drive Started.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double realTimeXSpeed = xSpeedFunction.get();
    double realTimeYSpeed = ySpeedFunction.get();
    double realTimeZRotation = rotationFunction.get();
    driveSubsystem.driveMotors(realTimeXSpeed, realTimeYSpeed, realTimeZRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Mechanum Drive Ended.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
