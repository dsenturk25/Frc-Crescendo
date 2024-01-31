package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** Ported from DriveForwardCmd */
public class TurnAroundCmd extends Command {

    private DriveSubsystem driveSubsystem;
    private double rotationSpeed;

    public TurnAroundCmd(DriveSubsystem driveSubsystem, double rotationSpeed){
        this.driveSubsystem = driveSubsystem;
        this.rotationSpeed = rotationSpeed;
        addRequirements();
    }

  @Override
  public void initialize() {
    System.out.println("TurnAroundCmd started!");
  }

  @Override
  public void execute() {
    driveSubsystem.driveMotors(0, 0, rotationSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveMotors(0, 0, 0);
    System.out.println("TurnAroundCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
