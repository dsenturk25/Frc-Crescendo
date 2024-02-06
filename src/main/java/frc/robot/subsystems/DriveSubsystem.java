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
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanumDriveConstants;
import frc.robot.Constants.AutonomousConstants;

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

  PIDController turnPidController = new PIDController(AutonomousConstants.kP_DRIVE, AutonomousConstants.kI_DRIVE, AutonomousConstants.kD_DRIVE);
  PIDController posePidController = new PIDController(AutonomousConstants.kP_TURN, AutonomousConstants.kI_TURN, AutonomousConstants.kD_TURN);

  ADIS16448_IMU gyro = new ADIS16448_IMU();

  public DriveSubsystem() {
    rightMotorFront.setInverted(true);
    rightMotorRear.setInverted(true);
    
    posePidController.setIZone(AutonomousConstants.iZone_TURN);
    turnPidController.setIZone(Units.degreesToRadians(AutonomousConstants.iZone_TURN));
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

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
    MecanumDriveWheelSpeeds wheelSpeeds = m_MecanumDriveKinematics.toWheelSpeeds(chassisSpeeds);

    double lf = wheelSpeeds.frontLeftMetersPerSecond + zRotation;
    double lr = wheelSpeeds.rearLeftMetersPerSecond - zRotation;

    double rf = wheelSpeeds.frontRightMetersPerSecond + zRotation;
    double rr = wheelSpeeds.rearRightMetersPerSecond - zRotation;
    
    leftMotorFront.set(lf);
    leftMotorRear.set(lr);

    rightMotorFront.set(rf);
    rightMotorRear.set(rr);
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

  public double getAbsMax(double[] arr) {  // Second Method
    double max = 0;

    for (double d : arr) {
      if (d < 0) max = -d >= max ? -d : max;
      else max = d >= max ? d : max;
    }
    return max;
  }

  public double[] normalizeSpeeds(double[] speedsArray) {
    double max = getAbsMax(speedsArray);

    if (max > 1) {
      for (int i = 0; i < speedsArray.length; i++) {
        speedsArray[i] /= max;
      }
    }

    return speedsArray;
  }

  public void driveMotorsManual(double xSpeed, double ySpeed, double zRotation) {  // Second Method
    double magnitude = getMagnitude(xSpeed, ySpeed);
    double angle = getAngle(xSpeed, ySpeed);

    double xComponent = magnitude * Math.cos(angle + Units.degreesToRadians(45));
    double yComponent = magnitude * Math.sin(angle + Units.degreesToRadians(45));

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

  
  public void driveMotorsDummy(double xSpeed, double ySpeed, double zRotation) {  // Fourth Method
    
    double lf = xSpeed + ySpeed + zRotation;
    double lr = xSpeed - ySpeed + zRotation;
    double rf = xSpeed + ySpeed - zRotation;
    double rr = xSpeed - ySpeed - zRotation;

    double[] speedsArray = {lf, lr, rf, rr};

    double[] normalizedArray = normalizeSpeeds(speedsArray);

    leftMotorFront.set(normalizedArray[0]);
    leftMotorRear.set(normalizedArray[1]);

    rightMotorFront.set(normalizedArray[2]);
    rightMotorRear.set(normalizedArray[3]);
  }

  public void driveMotorsMecanumSwerve(double xSpeed, double ySpeed, double zRotation) {
    double magnitude = getMagnitude(xSpeed, ySpeed);
    double angle = getAngle(xSpeed, ySpeed);

    double cartesianAngle = angle + Units.degreesToRadians(45);

    double xComponent = magnitude * Math.cos(cartesianAngle + gyro.getAngle());
    double yComponent = magnitude * Math.sin(cartesianAngle + gyro.getAngle());

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

  public void resetGyro() {
    gyro.reset();
  }
}
