package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {


    private Spark shootMotor = new Spark(ShooterConstants.SHOOT_MOTOR_PORT);
    
    public ShooterSubsystem() {
        shootMotor.setInverted(false);
    }

    public boolean exampleCondition() {
        return false;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
    
    public void setMotor(double speed){
        shootMotor.set(speed);
    }
}
