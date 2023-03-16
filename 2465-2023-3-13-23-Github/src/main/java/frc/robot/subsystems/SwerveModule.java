// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*Written by MBJ */

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class SwerveModule 
{
    private final CANSparkMax drive;
    private final CANSparkMax turn;

    private final RelativeEncoder driveEncoder;
    private final AbsoluteEncoder turnEncoder;

    private final SparkMaxPIDController drivePID;
    private final SparkMaxPIDController turnPID;

    private double class_chassis_offset = 0;
    private SwerveModuleState class_desired_state = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(CANSparkMax driver, CANSparkMax turner, double offset)
    {
        drive = driver;
        turn = turner;

        driveEncoder = drive.getEncoder();
        turnEncoder = turn.getAbsoluteEncoder(Type.kDutyCycle);
        drivePID = drive.getPIDController();
        turnPID = turn.getPIDController();
        drivePID.setFeedbackDevice(driveEncoder);
        turnPID.setFeedbackDevice(turnEncoder);
                
        driveEncoder.setPositionConversionFactor(ModuleConstants.DriveEncoderDistancePerPulse);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DriveEncoderDistancePerPulse/60);

        turnEncoder.setPositionConversionFactor(Math.PI * 2);
        turnEncoder.setVelocityConversionFactor((Math.PI * 2)/60);

        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI * 2);
        turnPID.setPositionPIDWrappingMinInput(0);

        drivePID.setP(ModuleConstants.kDriveP);
        drivePID.setI(ModuleConstants.kDriveI);
        drivePID.setD(ModuleConstants.kDriveD);
        drivePID.setFF(ModuleConstants.kDriveFF);
        drivePID.setOutputRange(ModuleConstants.kMinDriveOutput, ModuleConstants.kMaxDriveOutput);

        turnPID.setP(ModuleConstants.kTurnP);
        turnPID.setI(ModuleConstants.kTurnI);
        turnPID.setD(ModuleConstants.kTurnD);
        turnPID.setFF(ModuleConstants.kTurnFF);
        turnPID.setOutputRange(ModuleConstants.kMinTurnOutput, ModuleConstants.kMaxturnOutput);

        drive.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kBrake);

        drive.setSmartCurrentLimit(50);
        turn.setSmartCurrentLimit(20);

        class_chassis_offset = offset;
        class_desired_state.angle = new Rotation2d(turnEncoder.getPosition());
        driveEncoder.setPosition(0);

    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition() - class_chassis_offset));
    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(class_chassis_offset));
    
        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(turnEncoder.getPosition()));
    
        // Command driving and turning SPARKS MAX towards their respective setpoints.
        drivePID.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turnPID.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
    
        class_desired_state = desiredState;
      }
    
      public void resetEncoders() {
        driveEncoder.setPosition(0);
      }
}
