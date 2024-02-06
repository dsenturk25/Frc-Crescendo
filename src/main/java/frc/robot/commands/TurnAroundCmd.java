package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.AutonomousConstants;

/** Ported from DriveForwardCmd */
public class TurnAroundCmd extends Command {

  private DriveSubsystem driveSubsystem;
  private double rotationSpeed;
  private double startTime;
  private PhotonCamera photonCamera;
  private double latestResTimestamp;
  
  public TurnAroundCmd(DriveSubsystem driveSubsystem, double rotationSpeed){
      this.driveSubsystem = driveSubsystem;
      this.rotationSpeed = rotationSpeed;
      addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    this.startTime = Timer.getFPGATimestamp();
    System.out.println("TurnAroundCmd started!");
  }

  @Override
  public void execute() {
    if ((Timer.getFPGATimestamp() - startTime) < AutonomousConstants.TURN_AROUND_TIME) {
      driveSubsystem.driveMotors(0, 0, rotationSpeed);

      var objectPipelineRes = photonCamera.getLatestResult();
      double latestResultTimestamp = objectPipelineRes.getTimestampSeconds();

      if (objectPipelineRes.hasTargets() && latestResultTimestamp > this.latestResTimestamp) {
        this.latestResTimestamp = latestResultTimestamp;
        end(true);  // Will search till find
      }
    } else {
      new SequentialCommandGroup(
        new DriveForwardCmd(driveSubsystem, 0.2, 2.0),  // Execute after a 270° turn optimally
        new TurnAroundCmd(driveSubsystem, rotationSpeed)
      );
    }
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