package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AlignObjectCmd extends Command {

    DriveSubsystem driveSubsystem;
    PhotonCamera photonCamera;

    public AlignObjectCmd(DriveSubsystem driveSubsystem, PhotonCamera photonCamera){

        this.driveSubsystem = driveSubsystem;
        this.photonCamera = photonCamera;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AlignObjectCmd started!");
    }

    @Override
    public void execute() {
        var result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            var yaw = target.getYaw();
            driveSubsystem.turnPID(yaw);
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignObjectCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
