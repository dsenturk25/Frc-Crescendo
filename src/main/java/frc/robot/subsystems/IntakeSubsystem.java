package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private double[] armLevelMapping = {
        IntakeConstants.INTAKE_DEGREE,
        IntakeConstants.LOWER_THROW_DEGREE,
        IntakeConstants.CANON_FEEDER_DEGREE
    };
    
    private Spark armMotor = new Spark(IntakeConstants.ARM_MOTOR_PORT);

    private Spark intakeLeftMotor = new Spark(IntakeConstants.INTAKE_LEFTMOTOR_PORT );
    private Spark intakeRightMotor = new Spark(IntakeConstants.INTAKE_RIGHTMOTOR_PORT );

    private DigitalInput intakeLimitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_PORT);

    private PIDController m_pidController = new PIDController(
        IntakeConstants.kP, 
        IntakeConstants.kI,
        IntakeConstants.kD);
    
    private Encoder armEncoder = new Encoder(
        IntakeConstants.ENCODER_SOURCE_A, 
        IntakeConstants.ENCODER_SOURCE_B, 
        false, 
        EncodingType.k4X);

    public IntakeSubsystem() {
        m_pidController.setIZone(IntakeConstants.kI_LIMIT);  // in degrees
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

    public double getEncoderDegree() {
        return armEncoder.get() / IntakeConstants.K_ARM_TICK_2_DEG;
    }

    public void setArmAngle(int setpointIndex) {
        armMotor.set(m_pidController.calculate(getEncoderDegree(), armLevelMapping[setpointIndex]));
    }

    public void setIntake(boolean isTaking){

        if(isTaking){
            intakeLeftMotor.set(0.7);
            intakeRightMotor.set(0.7);
        }
        else{
            intakeLeftMotor.set(-0.7);
            intakeRightMotor.set(-0.7);
        }
    }

    public Boolean getLimitSwitchStatus () {
        return intakeLimitSwitch.get();
    }
}
