// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanumDriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private Spark leftMotorFront = new Spark(MechanumDriveConstants.LEFT_MOTOR_FRONT_PORT);
  private Spark leftMotorRear = new Spark(MechanumDriveConstants.LEFT_MOTOR_REAR_PORT);
  private Spark rightMotorFront = new Spark(MechanumDriveConstants.RIGHT_MOTOR_FRONT_PORT);
  private Spark rightMotorRear = new Spark(MechanumDriveConstants.RIGHT_MOTOR_REAR_PORT);

  Translation2d m_frontLeftLocation = new Translation2d(MechanumDriveConstants.MECHANUM_WHEEL_LOCATION, MechanumDriveConstants.MECHANUM_WHEEL_LOCATION);
  Translation2d m_frontRightLocation = new Translation2d(MechanumDriveConstants.MECHANUM_WHEEL_LOCATION, -MechanumDriveConstants.MECHANUM_WHEEL_LOCATION);
  Translation2d m_backLeftLocation = new Translation2d(-MechanumDriveConstants.MECHANUM_WHEEL_LOCATION, MechanumDriveConstants.MECHANUM_WHEEL_LOCATION);
  Translation2d m_backRightLocation = new Translation2d(-MechanumDriveConstants.MECHANUM_WHEEL_LOCATION, -MechanumDriveConstants.MECHANUM_WHEEL_LOCATION);


  MecanumDriveKinematics m_MecanumDriveKinematics = new MecanumDriveKinematics(m_frontRightLocation, m_frontLeftLocation, m_backRightLocation, m_backLeftLocation);

  public DriveSubsystem() {
    rightMotorFront.setInverted(true);
    rightMotorRear.setInverted(true);  
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

  public void driveMotors(double xSpeed, double ySpeed, double zRotation) {

    double rotationToRad = zRotation * Math.PI;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationToRad);
    MecanumDriveWheelSpeeds wheelSpeeds = m_MecanumDriveKinematics.toWheelSpeeds(chassisSpeeds);

    leftMotorFront.set(wheelSpeeds.frontLeftMetersPerSecond);
    leftMotorRear.set(wheelSpeeds.rearLeftMetersPerSecond);
    rightMotorFront.set(wheelSpeeds.frontRightMetersPerSecond);
    rightMotorRear.set(wheelSpeeds.rearRightMetersPerSecond);
  }
}
