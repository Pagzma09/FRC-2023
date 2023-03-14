// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.GoToLiftStates;

public class GoToLiftEncoder extends CommandBase {
  private final Lift lifter = Robot.lift;
  private final GoToLiftStates state;
  private boolean isDone = false;
  /** Creates a new GoToLiftEncoder. */
  public GoToLiftEncoder(GoToLiftStates state) {
    this.state = state;
    addRequirements(lifter);
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
      case HIGH:
      if(lifter.highLimit())
      {
      lifter.runPosition(285);
      }
      else
      {
        isDone = true;
      }
      break;

      case MIDDLE:
      if(lifter.midLimit())
      {
      lifter.runPosition(160);
      }
      else
      {
        isDone = true;
      }
      break;

      case LOW:
      if(lifter.lowLimit())
      {
      lifter.runPosition(10);
      }
      else
      {
        isDone = true;
      }
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
