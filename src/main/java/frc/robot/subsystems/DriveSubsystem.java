// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
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

  MecanumDrive m_Drive = new MecanumDrive(leftMotorFront, leftMotorRear, rightMotorFront, rightMotorRear);

  PIDController turnPidController = new PIDController(0, 0, 0);
  PIDController posePidController = new PIDController(0, 0, 0);

  public DriveSubsystem() {
    rightMotorFront.setInverted(true);
    rightMotorRear.setInverted(true);
    
    posePidController.setIZone(0.2);
    turnPidController.setIZone(Units.degreesToRadians(5));
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

  public void driveMotors(double xSpeed, double ySpeed, double zRotation) {  // First Method: Most optimal

    double rotationToRad = zRotation * Math.PI;

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotationToRad);
    double mFL = chassisSpeeds.vyMetersPerSecond + chassisSpeeds.vxMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
    double mFR = chassisSpeeds.vyMetersPerSecond + chassisSpeeds.vxMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
    double mRL = chassisSpeeds.vyMetersPerSecond - chassisSpeeds.vxMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond;
    double mRR = chassisSpeeds.vyMetersPerSecond - chassisSpeeds.vxMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond;
    MecanumDriveWheelSpeeds wheelSpeeds = m_MecanumDriveKinematics.toWheelSpeeds(chassisSpeeds);

    if(Math.abs(chassisSpeeds.omegaRadiansPerSecond) > 0.5){
      leftMotorFront.set(mFL * 0.5);
      leftMotorRear.set(mRL * 0.5);
      rightMotorFront.set(mFR * 0.5);
      rightMotorRear.set(mRR * 0.5);
    }
    else{
      leftMotorFront.set(wheelSpeeds.frontLeftMetersPerSecond * 0.5);
      leftMotorRear.set(wheelSpeeds.rearLeftMetersPerSecond * 0.5);
      rightMotorFront.set(wheelSpeeds.frontRightMetersPerSecond * 0.5);
      rightMotorRear.set(wheelSpeeds.rearRightMetersPerSecond * 0.5);
    }
  }

  public void turnPID(double yaw) {
    driveMotors(0, 0, turnPidController.calculate(0, yaw));
  }

  public void drivePID(double x, double y, double angle) {
    driveMotors(
      posePidController.calculate(0, x), 
      posePidController.calculate(0, y), 
      turnPidController.calculate(0, angle));
  }


  public double getMagnitude(double xSpeed, double ySpeed) {  // Second Method
    return Math.hypot(xSpeed, ySpeed);
  }

  public double getAngle(double xSpeed, double ySpeed) {  // Second Method
    return Math.atan2(ySpeed, xSpeed);
  }

  public double getMax(double[] arr) {  // Second Method
    double max = 0;

    for (double d : arr) {
      max = d >= max ? d : max;
    }
    return max;
  }

  public double[] normalizeSpeeds(double[] speedsArray) {
    double max = getMax(speedsArray);
    for (int i = 0; i < speedsArray.length; i++) {
      speedsArray[i] /= max;
    }

    return speedsArray;
  }

  public void driveMotorsManual(double xSpeed, double ySpeed, double zRotation) {  // Second Method
    double magnitude = getMagnitude(xSpeed, ySpeed);
    double angle = getAngle(xSpeed, ySpeed);

    double xComponent = magnitude * Math.cos(angle - Units.degreesToRadians(45));
    double yComponent = magnitude * Math.sin(angle - Units.degreesToRadians(45));

    double threshold = MechanumDriveConstants.SPEED_THRESHOLD;

    double lf = (xComponent * threshold) + zRotation;
    double rr = (xComponent * threshold) - zRotation;
    double lr = (yComponent * threshold) + zRotation;
    double rf = (yComponent * threshold) - zRotation;

    double[] speedsArray = {lf, lr, rf, rr};

    double[] normalizedArray = normalizeSpeeds(speedsArray);

    leftMotorFront.set(normalizedArray[0]);
    leftMotorRear.set(normalizedArray[1]);

    rightMotorFront.set(normalizedArray[2]);
    rightMotorRear.set(normalizedArray[3]);
  }

  
  public void driveMotorsCartesian(double xSpeed, double ySpeed, double zRotation) {  // Third Method
    m_Drive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

}
