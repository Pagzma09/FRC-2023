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

public class AutoBalanceV2 extends CommandBase {
  /** Creates a new AutoBalance. */
  private final Drive driver = Robot.drive;
  private final double desired_pitch = 3.5;
  private boolean isDone;
  double power = 0.00001;
  double error = 0;
  double reportederror = 1;
  public AutoBalanceV2() {
    addRequirements(driver);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    power = 0.00001;
    reportederror = 1;
    error = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //-expl Set error to pitch
    if(Math.abs(driver.getPitch()) > desired_pitch)
    {   
    error = Math.abs(driver.getPitch()) - desired_pitch;
    reportederror = error;
    
    //-expl Clip power to 0 - 0.13. Multiply power by error.
    if(error < 0)
    {
      power = 0;
    }
    else
    {
      power = power * error;
    }

    if(power > 0.13)
    {
      power = 0.13;
    }

    //-expl Update SmartDashboard
    SmartDashboard.putNumber("command pitch", driver.getPitch());
    SmartDashboard.putNumber("power", power);
    SmartDashboard.putNumber("error", error);

    //-expl Drive with power set earlier

    //-expl Drive forward if pitch is negative
    //-expl Drive backward if pitch is positive
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
    if(reportederror < 0.5)
    {
    return true;
    }
    else
    {
    return false;
    }
  }
}
