// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ExtensionConstants;

public class Extension extends SubsystemBase {
  /** Creates a new Extension. */
  private final CANSparkMax extensionmotor = Constants.extension;
  private final DigitalInput forwardlimit = Constants.forwardlimit;
  private final DigitalInput reverselimit = Constants.reverselimit;
  private final SparkMaxPIDController extensionPID;
  public boolean extension_enc_troubleshooter = false;
  public static enum ExtensionBasicStates {OUT, IN, STOP};
  private static double holder = 0;

  public Extension() {
    extensionmotor.restoreFactoryDefaults();

    extensionPID = extensionmotor.getPIDController();
    extensionPID.setP(0.0005);
    extensionPID.setI(0);
    extensionPID.setD(ExtensionConstants.extensionD);
    extensionPID.setFF(0.0005);
    extensionPID.setOutputRange(-0.9, 0.9);
    extensionPID.setFeedbackDevice(extensionmotor.getEncoder());

    
    extensionPID.setSmartMotionMaxVelocity(5000, 0);
    extensionPID.setSmartMotionMinOutputVelocity(1, 0);
    extensionPID.setSmartMotionMaxAccel(1500, 0);
    extensionPID.setSmartMotionAllowedClosedLoopError(1, 0);
    
    //extensionmotor.getEncoder().setInverted(true);
    extensionmotor.getEncoder().setPositionConversionFactor(1);
    extensionmotor.getEncoder().setPosition(0);
    holder = extensionmotor.getEncoder().getPosition();

    extensionmotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power)
  {
    extensionmotor.set(power);
  }

  public void setPosition(double position)
  {
    holder = position;
    extensionPID.setReference(holder, ControlType.kSmartMotion);
  }

  public double returnPIDError(double desired_pos)
  {
    double error = Math.abs(extensionmotor.getEncoder().getPosition()) - desired_pos;
    return error;
  }

  public void SmartDashValues()
  {
    SmartDashboard.putBoolean("Forward Limit", forwardlimit.get());
    SmartDashboard.putBoolean("Reverse Limit", reverselimit.get());
    SmartDashboard.putNumber("Extension Position", extensionmotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("Extension Command Troubleshooter", extension_enc_troubleshooter);
    SmartDashboard.putNumber("Position Conversion Factor", extensionmotor.getEncoder().getPositionConversionFactor());
    SmartDashboard.putNumber("holder", holder);
  }
}
