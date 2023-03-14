// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.mult.SubmatrixOps_DDRM;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.wristConstants;

public class Wrist extends SubsystemBase {
  private final CANSparkMax wristMotor = Constants.wrist;
  private final SparkMaxPIDController wristPID;
  private final RelativeEncoder wristCoder;
  public double desired_wrist_pos;
  public static enum WristBasicStates {OUT, IN, HOLD};
  /** Creates a new Wrist. */
  public Wrist() {
    wristPID = wristMotor.getPIDController();
    wristCoder = wristMotor.getEncoder();
    
    wristPID.setP(wristConstants.wristP);
    wristPID.setI(wristConstants.wristI);
    wristPID.setD(wristConstants.wristD);
    wristPID.setOutputRange(wristConstants.wristMinOut, wristConstants.wristMaxOut);
    //wristPID.setFeedbackDevice(wristCoder);
    //wristPID.setPositionPIDWrappingEnabled(false);

   // wristCoder.setPosition(0);

    desired_wrist_pos = wristCoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPos(double counts_to_move)
  {
    desired_wrist_pos = desired_wrist_pos + counts_to_move;

    /* 
    if(desired_wrist_pos > 0.4)
    {
      desired_wrist_pos = 0.4;
    } 
    else if(desired_wrist_pos < 0)
    {
      desired_wrist_pos = 0;
    } 
    */

   // wristPID.setReference(desired_wrist_pos, ControlType.kPosition);
  }

  /* 
  public void hold()
  {
    if(desired_wrist_pos != 0)
    {
      wristPID.setReference(desired_wrist_pos, ControlType.kPosition);
    }
    else
    {
      wristMotor.set(0);
    }
  }
  */

  public double getPosition()
  {
    return wristCoder.getPosition();
  }

  public void goToPosition(double Position)
  {
    /* 
    if(Position > 0.4)
    {
      Position = 0.4;
    }
    else if(Position < 0)
    {
      Position = 0.02;
    }
    */
    
   // wristPID.setReference(Position, ControlType.kPosition);
    //desired_wrist_pos = Position;
  }

  public void SmartDashValues()
  {
   
  }
}
