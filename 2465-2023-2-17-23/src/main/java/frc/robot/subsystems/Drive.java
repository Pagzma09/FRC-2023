// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*Written by MBJ */

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class Drive extends SubsystemBase {

  private final SwerveModule frontRight = new SwerveModule
  (Constants.frontRightDrive, 
  Constants.frontRightRotate, 
  0);

  private final SwerveModule frontLeft = new SwerveModule
  (Constants.frontLeftDrive, 
  Constants.frontLeftRotate, 
  0);

  private final SwerveModule rearRight = new SwerveModule
  (Constants.rearRightDrive, 
  Constants.rearRightRotate, 
  0);

  private final SwerveModule rearLeft = new SwerveModule
  (Constants.rearLeftDrive, 
  Constants.rearLeftRotate, 
  0);

  private final AHRS gyro = Constants.imu;

  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magRateLimiter = new SlewRateLimiter(1.8);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1.5);
  private double directionalSlewRate = 1.2;
  private double prevTime = WPIUtilJNI.now();


  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
   Constants.kinematics, 
   gyro.getRotation2d(),
   new SwerveModulePosition[]
   {
     frontLeft.getPosition(),
     frontRight.getPosition(),
     rearLeft.getPosition(),
     rearRight.getPosition()
   });

  /** Creates a new Drive. */
  public Drive() {
    
  }

  @Override
  public void periodic() {

    odometry.update(
      gyro.getRotation2d(), 
      new SwerveModulePosition[]
      {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        rearLeft.getPosition(),
        rearRight.getPosition()
      });
    // This method will be called once per scheduler run
  }

  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose)
  {
    odometry.resetPosition(
      gyro.getRotation2d(),
      new SwerveModulePosition[]
   {
     frontLeft.getPosition(),
     frontRight.getPosition(),
     rearLeft.getPosition(),
     rearRight.getPosition()
   }, pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        Constants.kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void slewdrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(directionalSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magRateLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magRateLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magRateLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magRateLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = Constants.kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    rearLeft.resetEncoders();
    rearRight.resetEncoders();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getPitch()
  {
    return gyro.getPitch();
  }

  public double getTurnRate() {
    return gyro.getRate() * (false ? -1.0 : 1.0);
  }

  public void SmartDashValues()
  {
    SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odometry Angle", odometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Pitch", getPitch());
    SmartDashboard.putNumber("Odometry X Inches", Units.metersToInches(odometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("Odometry Y Inches", Units.metersToInches(odometry.getPoseMeters().getY()));
    //SmartDashboard.putNumber("Max Radians", odometry.getPoseMeters().)
  }

  public void Stop()
  {
    Constants.frontRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.frontLeftDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearLeftDrive.setIdleMode(IdleMode.kBrake);

    drive(0.02, 0, 0, false);

    drive(0, 0, 0, false);
    
    return;
  }

}
