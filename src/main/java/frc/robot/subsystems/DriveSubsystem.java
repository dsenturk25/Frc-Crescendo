// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanumDriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private Spark leftMotorFront = new Spark(MechanumDriveConstants.LEFT_MOTOR_FRONT_PORT);
  private Spark leftMotorRear = new Spark(MechanumDriveConstants.LEFT_MOTOR_REAR_PORT);
  private Spark rightMotorFront = new Spark(MechanumDriveConstants.RIGHT_MOTOR_FRONT_PORT);
  private Spark rightMotorRear = new Spark(MechanumDriveConstants.RIGHT_MOTOR_REAR_PORT);

  private MecanumDrive m_Drive = new MecanumDrive(leftMotorFront, leftMotorRear, rightMotorFront, rightMotorRear);

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {}

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void driveMotors(double xSpeed, double ySpeed, double zRotation) {
    m_Drive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

}
