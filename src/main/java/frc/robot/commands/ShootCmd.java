
package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<Double> speedFunction;

    public ShootCmd(ShooterSubsystem shooterSubsystem, Supplier<Double> speedFunction){
        this.shooterSubsystem = shooterSubsystem;
        this.speedFunction = speedFunction;
        addRequirements(shooterSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("ShootCmd started!");
  }

  @Override
  public void execute() {
    shooterSubsystem.setMotor(speedFunction.get());
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setMotor(0);
    System.out.println("ShootCmd ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}