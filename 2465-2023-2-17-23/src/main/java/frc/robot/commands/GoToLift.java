// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.GoToLiftStates;

public class GoToLift extends CommandBase {
  //-expl Init lift, state, power, and variable 'done'
  private final Lift lifter = Robot.lift;
  private final GoToLiftStates states;
  private double power;
  private boolean done = false;
  /** Creates a new GoToLift. */
  public GoToLift(GoToLiftStates states, double power) {
    this.states = states;
    this.power = power;
    addRequirements(lifter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //-expl Switch over state.

    switch(states)
    {
      case HIGH:

      //-expl If state is high, set power to 0.5 until the top limit switch triggers.

      if(lifter.highLimit())
      {
        lifter.setPower(power);
      }
      else
      {
        done = true;
      }
      break;

      //-expl If state is low, set power to 0.5 until the bottom limit switch triggers.
      case LOW:
      if(lifter.lowLimit())
      {
        lifter.setPower(-power);
      }
      else
      {
        done = true;
      }
      break;

      //-expl If state is middle, use realtime position to center on the 180-150 range.
      case MIDDLE:
      if(lifter.midLimit())
      {
       if(lifter.getPos() > 180)
        {
          power = -power;
          lifter.setPower(power);
        }
      else if(lifter.getPos() < 150)
        {
          lifter.setPower(power);
        }
      else
        {
          done = true;
        }
      }
      else
      {
      done = true;
      }
      break;

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lifter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(done == true)
    {
      return true;
    }
    else
    {
    return false;
    }
  }
}
