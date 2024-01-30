package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;;

public class ClimbSubsystem extends SubsystemBase {

    private Spark climbMotor = new Spark(ClimbConstants.CLIMB_MOTOR_PORT);

    public ClimbSubsystem() {
        climbMotor.setInverted(false);
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
        climbMotor.set(speed);
    }
}