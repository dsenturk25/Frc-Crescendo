package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeArmCmd extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final int setpointIndex;

    public IntakeArmCmd(IntakeSubsystem intakeSubsystem, int setpointIndex) {
        this.intakeSubsystem = intakeSubsystem;
        this.setpointIndex = setpointIndex;

        addRequirements();
    }

    @Override
    public void initialize() {
        
        if (setpointIndex < 0 || setpointIndex > 2) {
            System.out.println("Error: setpoint out of range");
            end(true);
        }
        System.out.println("Intake Arm Command Started.");
    }

    @Override
    public void execute() {
        intakeSubsystem.setArmAngle(setpointIndex);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake Arm Command Ended.");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
