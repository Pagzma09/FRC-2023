// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.liftConstants;

public class Lift extends SubsystemBase {
  /** Creates a new Lift. */

  private final CANSparkMax liftmotor = Constants.lift;
  private final DigitalInput lowestlevel = Constants.Level1;
  private final DigitalInput middlelevel = Constants.Level2;
  private final DigitalInput highestlevel = Constants.Level3;
  public double hold_level;
  public static enum LiftBasicStates {UP, DOWN, STOP};
  public static enum GoToLiftStates {HIGH, MIDDLE, LOW}
  private final RelativeEncoder liftmotorcoder = Constants.lift.getEncoder();
  private final SparkMaxPIDController liftPID = Constants.lift.getPIDController();
  //public boolean V2handler = false;
  public double V2holder;
  
  public Lift() {
   liftPID.setP(0.00001);
   liftPID.setI(liftConstants.liftI);
   liftPID.setD(0.0000);
   liftPID.setFF(0.005);
   liftPID.setOutputRange(-0.4, 0.7);
   liftPID.setFeedbackDevice(liftmotorcoder);

   liftPID.setSmartMotionMaxVelocity(4750, 0);
   liftPID.setSmartMotionMinOutputVelocity(1, 0);
   liftPID.setSmartMotionMaxAccel(900, 0);
   liftPID.setSmartMotionAllowedClosedLoopError(5, 0);

   liftmotorcoder.setPosition(0);

   V2holder = 0;
   //hold_level = 0;
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }

  public void setPower(double setPower)
  {
    if(setPower != 0)
    {
      V2holder = getPos();
    }

    liftmotor.set(setPower);
  }

  public void runPosition(double position)
  {
    liftPID.setReference(position, ControlType.kPosition);
  }

  public void SmartDashValues()
  {
    SmartDashboard.putBoolean("Level 1", lowestlevel.get());
    SmartDashboard.putBoolean("Level 2", middlelevel.get());
    SmartDashboard.putBoolean("Level 3", highestlevel.get());
    SmartDashboard.putNumber("Lift Position", getPos());
    SmartDashboard.putNumber("LiftHolder", V2holder);
    //SmartDashboard.putBoolean("LiftHandler", V2handler);
  }

  public boolean lowLimit()
  {
    return lowestlevel.get();
  }
  
  public boolean midLimit()
  {
    return middlelevel.get();
  }

  public boolean highLimit()
  {
    return highestlevel.get();
  }

  public double getPos()
  {
    return liftmotorcoder.getPosition();
  }

  /* 
  public void setPos(double position)
  {
    //liftmotorcoder.setPosition(position);
  }
  */

  public void setSmartPos(double position)
  {
    V2holder = position;
    liftPID.setReference(V2holder, ControlType.kSmartMotion);
  }

  public double returnPIDError(double pos)
  {
    double returner = Math.abs(pos - liftmotorcoder.getPosition());
    return returner;
  }

}
