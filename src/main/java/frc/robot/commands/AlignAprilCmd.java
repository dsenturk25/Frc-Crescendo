package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PoseSubsystem;

public class AlignAprilCmd extends Command {

    PoseSubsystem poseSubsystem;

    public AlignAprilCmd(PoseSubsystem poseSubsystem){

        this.poseSubsystem = poseSubsystem;
        addRequirements(poseSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("AlignAprilCmd started!");
    }

    @Override
    public void execute() {
        poseSubsystem.poseDrive();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignAprilCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
