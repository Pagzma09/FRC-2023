// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.clawConstants;

public class ClawAbsoluteEncoder extends SubsystemBase {
  /** Creates a new ClawAbsoluteEncoder. */
  private final CANSparkMax claw = Constants.ClawRotate;
  private final SparkMaxAbsoluteEncoder clawcoder;
  private final SparkMaxPIDController clawPID;
  public double desired_position;
  public ClawAbsoluteEncoder() {
    clawcoder = claw.getAbsoluteEncoder(Type.kDutyCycle);
    clawcoder.setInverted(false);

    clawPID = claw.getPIDController();
    clawPID.setP(0.05);
    clawPID.setI(clawConstants.clawRotateI);
    clawPID.setD(clawConstants.clawRotateD);
    clawPID.setOutputRange(clawConstants.clawMinOut, clawConstants.clawMaxOut);
    clawPID.setPositionPIDWrappingEnabled(false);
    clawPID.setPositionPIDWrappingMaxInput(0.3);
    clawPID.setPositionPIDWrappingMinInput(0);
    clawPID.setFeedbackDevice(clawcoder);

    //clawPID.setSmartMotionAllowedClosedLoopError(0.01, 0);
    
    desired_position = clawcoder.getPosition();

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("CAE POS", getPosition());
    SmartDashboard.putNumber("CAE DESIRED", desired_position);
    // This method will be called once per scheduler run
  }

  public void goToPosition(double position)
  {
    desired_position = position;
    clawPID.setReference(desired_position, CANSparkMax.ControlType.kPosition);
  }

  public double getPosition()
  {
    return clawcoder.getPosition();
  }

}
