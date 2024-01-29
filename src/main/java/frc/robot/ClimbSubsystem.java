package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    // Motor
    private Spark climbMotor = new Spark(); //parantezin icinde bir sey olmali ama ne olmali bilmiyorum.


    public ClimbSubsystem() {}

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

    // Motorları çalıştıran fonksiyon
    
}
