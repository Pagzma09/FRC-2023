// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*Written by MBJ */

package frc.robot.subsystems;

import java.io.Externalizable;
import java.security.PublicKey;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.clawConstants;

public class Claw extends SubsystemBase {
  private final CANSparkMax clawRotate = Constants.ClawRotate;
  private final TalonFX clawSpin1 = Constants.ClawSpin1;
  private final TalonFX clawSpin2 = Constants.ClawSpin2;
  private final SparkMaxPIDController clawRotatePID;
  public double clawDesiredPosition;
  private final RelativeEncoder clawEncoder;
  public static enum ClawBasicStates {Suck, Spit, Open, Close, Stop}
  /** Creates a new Claw. */
  public Claw() {
    clawEncoder = clawRotate.getEncoder();
    //clawEncoder.setInverted(true);

    clawRotatePID = clawRotate.getPIDController();
    clawRotatePID.setP(clawConstants.clawRotateP);
    clawRotatePID.setI(clawConstants.clawRotateI);
    clawRotatePID.setD(clawConstants.clawRotateD);
    clawRotatePID.setOutputRange(clawConstants.clawMinOut, clawConstants.clawMaxOut);
   // clawRotatePID.setPositionPIDWrappingEnabled(true);
   // clawRotatePID.setPositionPIDWrappingMaxInput(0.4);
   // clawRotatePID.setPositionPIDWrappingMinInput(0);
   // clawRotatePID.setFeedbackDevice(clawEncoder);
  

    clawDesiredPosition = clawEncoder.getPosition();
    clawEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpin(double power)
  {
    clawSpin1.set(ControlMode.PercentOutput, power);
    clawSpin2.set(ControlMode.PercentOutput, power);

  
  }

  public void setRotate(double counts_to_move)
  {
    clawDesiredPosition = clawDesiredPosition + counts_to_move;

    if(clawDesiredPosition > 0.4)
    {
      clawDesiredPosition = 0.4;
    }
    else if(clawDesiredPosition < 0)
    {
      clawDesiredPosition = 0;
    }

    clawRotatePID.setReference(clawDesiredPosition, ControlType.kPosition);
  }

  public void goToRotate(double position)
  {
    /* 
    if(position > 0.4)
    {
      position = 0.4;
    }
    else if(position < 0)
    {
      position = 0;
    }
    */
    

    clawDesiredPosition = position;
    clawRotatePID.setReference(position, ControlType.kPosition);
  }

  public void hold()
  {
    if(clawDesiredPosition != 0)
    {
      clawRotatePID.setReference(clawDesiredPosition, ControlType.kPosition);
    }
    else
    {
      clawRotate.set(0);
    }
  }

  public double getPosition()
  {
    return clawEncoder.getPosition();
  }

  public void SmartDashValues()
  {
    //SmartDashboard.putNumber("Claw Rotate Position", clawEncoder.getPosition());
    //SmartDashboard.putNumber("Desired Claw Position", clawDesiredPosition);
  }
}
