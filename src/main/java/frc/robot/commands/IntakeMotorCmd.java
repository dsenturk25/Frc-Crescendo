package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeMotorCmd extends Command {

    private final boolean isTaking;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeMotorCmd(IntakeSubsystem intakeSubsystem, boolean isTaking) {
        this.intakeSubsystem = intakeSubsystem;
        this.isTaking=isTaking;
        addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize() {
        System.out.println("Intake Motor Joystick started");
    }

    @Override
    public void execute() {
        intakeSubsystem.setIntake(isTaking);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}