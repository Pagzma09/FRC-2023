// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.wristConstants;

public class WristAbsoluteEncoder extends SubsystemBase {
  /** Creates a new WristAbsoluteEncoder. */
  private final CANSparkMax wrist = Constants.wrist;
  private final SparkMaxAbsoluteEncoder wristcoder;
  private final SparkMaxPIDController wristPID;
  public double desired_position;

  public WristAbsoluteEncoder() {
    wrist.restoreFactoryDefaults();
    wrist.setInverted(true);
    wristcoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
    wrist.setInverted(true);
    //clawcoder.setPositionConversionFactor(360);

    wristPID = wrist.getPIDController();
    wristPID.setP(0.0001);
    wristPID.setI(wristConstants.wristI);
    wristPID.setD(wristConstants.wristD);
    wristPID.setFF(0.005);
    wristPID.setOutputRange(-0.7, 0.7);
    wristPID.setPositionPIDWrappingEnabled(true);
    wristPID.setPositionPIDWrappingMaxInput(1);
    wristPID.setPositionPIDWrappingMinInput(0);
    wristPID.setFeedbackDevice(wristcoder);

    //clawPID.setSmartMotionAllowedClosedLoopError(0.01, 0)
    wristPID.setSmartMotionMaxVelocity(2000, 0);
    wristPID.setSmartMotionMinOutputVelocity(1, 0);
    wristPID.setSmartMotionMaxAccel(300, 0);
    wristPID.setSmartMotionAllowedClosedLoopError(0.01, 0);
    
    desired_position = wristcoder.getPosition();
    wrist.burnFlash();

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("WAE POS", getPosition());
    SmartDashboard.putNumber("WAE DESIRED", desired_position);
    // This method will be called once per scheduler run
  }

  public void goToPosition(double position)
  {
    desired_position = position;
    wristPID.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public double getPosition()
  {
    return wristcoder.getPosition();
  }
}
