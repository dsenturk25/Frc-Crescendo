
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private final double speed;

    public ShootCmd(ShooterSubsystem shooterSubsystem, double speed){
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        addRequirements(shooterSubsystem);
    }

  @Override
  public void initialize() {
    System.out.println("ShootCmd started!");
  }

  @Override
  public void execute() {
    shooterSubsystem.setMotor(speed);
    if(true){ //burasi olmadi
        shooterSubsystem.setMotor(ShooterConstants.LOW_SHOOT_SPEED);
        System.out.println("Robot is shooting higher");
    }
    else if(false){ //burasi olmadi
        shooterSubsystem.setMotor(ShooterConstants.HIGH_SHOOT_SPEED);
        System.out.println("Robot is shooting Lower");
    }

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