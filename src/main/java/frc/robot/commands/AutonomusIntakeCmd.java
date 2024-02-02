package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.Constants.AutonomousConstants;

public class AutonomusIntakeCmd extends Command {
    
    PhotonCamera photonCamera;
    IntakeSubsystem intakeSubsystem;
    DriveSubsystem driveSubsystem;
    double beforeIntakeTimestamp, latestResultTimeStamp;

    public AutonomusIntakeCmd(PhotonCamera photonCamera, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem){    
        
        this.photonCamera = photonCamera;
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AutonomusIntakeCmd started!");
    }

    @Override
    public void execute() {

        var pipelineResult = photonCamera.getLatestResult();

        if (pipelineResult.hasTargets()) {

        var objectTarget = pipelineResult.getBestTarget();

        double pipelineResultTimestamp = pipelineResult.getTimestampSeconds();

            if (
                objectTarget.getArea() /* Returns the percent */ > AutonomousConstants.MAX_CONTOUR_AREA_PERCENTAGE
                && pipelineResultTimestamp > latestResultTimeStamp
            ) {
                this.latestResultTimeStamp = pipelineResultTimestamp;

                driveSubsystem.driveMotors(AutonomousConstants.INTAKE_DRIVE_FORWARD_SPEED * 0.25, 0, 0);
                if ((Timer.getFPGATimestamp() - this.beforeIntakeTimestamp) > AutonomousConstants.INTAKE_TIME) {
                    intakeSubsystem.setIntake(false);
                } else {
                    intakeSubsystem.setIntake(true);
                }
            } else {
                // Drive till get close a certain distance
                this.beforeIntakeTimestamp = Timer.getFPGATimestamp();
                driveSubsystem.driveMotors(AutonomousConstants.INTAKE_DRIVE_FORWARD_SPEED, 0, 0);
            } 

        } else {
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AutonomusIntakeCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
