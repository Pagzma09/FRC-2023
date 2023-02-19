// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final Drive driver = Robot.drive;
  private final double desired_pitch = 3.5;
  private boolean isDone;
  public AutoBalance() {
    addRequirements(driver);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //-expl Initalize variables
    double power = 0.00001;
    double error = 0;

    //-expl Calculate error
    while(Math.abs(driver.getPitch()) > desired_pitch)
    {   
    error = Math.abs(driver.getPitch()) - desired_pitch;

    //-expl Calculate power (if error is less than zero, something is wrong, so we set power to 0)
    if(error < 0)
    {
      power = 0;
    }
    else
    {
      power = power * error;
    }

    //-expl Clip the power between 0 and 0.13
    if(power > 0.13)
    {
      power = 0.13;
    }

    //-expl Update dashboard information
    SmartDashboard.putNumber("command pitch", driver.getPitch());
    SmartDashboard.putNumber("power", power);
    SmartDashboard.putNumber("error", error);

    //-expl Determine whether to drive forward or backward based on pitch
    //-expl If pitch is positive, drive backward
    //-expl If pitch is negative, drive forward (a little confusing)
    if(driver.getPitch() > 0)
    {
      driver.drive
      (-power, 
      0, 
      0, 
      false);
    }
    else if(driver.getPitch() < 0)
    {
      driver.drive
      (power, 
      0, 
      0, 
      false);
    }
  }  

    isFinished();

  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driver.drive
      (0, 
      0.05, 
      0, 
      false);

    Constants.frontRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.frontLeftDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearLeftDrive.setIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return true;
  }
}
