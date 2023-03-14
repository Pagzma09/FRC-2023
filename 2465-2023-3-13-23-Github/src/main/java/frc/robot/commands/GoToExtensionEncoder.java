// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;

public class GoToExtensionEncoder extends CommandBase {
  /** Creates a new GoToExtensionEncoder. */
  private final Extension extender = Robot.extension;
  private final Lift lifter = Robot.lift;
  private boolean kickback_error = false;
  private final double pos;
  public GoToExtensionEncoder(double pos) {
    this.pos = pos;
    addRequirements(extender);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extender.extension_enc_troubleshooter = false;
    kickback_error = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lifter.getPos() > 20)
    {
    extender.setPosition(pos);
    }
    else
    {
      DriverStation.reportError("KICKBACK! PLEASE RAISE THE LIFT!", kickback_error);
      kickback_error = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.setPosition(extender.getPosition());
    extender.extension_enc_troubleshooter = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    if(pos > 200 && !extender.getForward())
    {
      return true;
    }
    else if(pos < 50 && !extender.getReverse())
    {
      return true;
    }
    
    if(extender.returnPIDError(pos) < 2 || kickback_error)
    {
      return true;
    }
    else
    {
      return false; 
    }
  }
}
