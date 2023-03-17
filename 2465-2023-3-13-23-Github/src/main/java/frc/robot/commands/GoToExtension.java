// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.ExtensionBasicStates;

public class GoToExtension extends CommandBase {
  /** Creates a new GoToExtension. */
  private final Extension extensioner = Robot.extension;
  private final ExtensionBasicStates state;
  private final double power;
  private boolean isDone = false;
  public GoToExtension(ExtensionBasicStates state, double power) {
    this.state = state;
    this.power = power;
    addRequirements(extensioner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state)
    {
      case OUT:
      if(extensioner.getForward())
      {
        extensioner.setPower(power);
      }
      else
      {
        extensioner.setPower(0);
        isDone = true;
      }
      break;

      case IN:
      if(extensioner.getReverse())
      {
        extensioner.setPower(-power);
      }
      else
      {
        extensioner.setPower(0);
        isDone = true;
      }
      break;

      case STOP:
      extensioner.setPower(0);
      isDone = true;
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone)
    {
      return true;
    }
    else
    {
    return false;
    }
  }
}
