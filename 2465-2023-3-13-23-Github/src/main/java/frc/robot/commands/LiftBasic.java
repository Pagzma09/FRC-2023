// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lift.LiftBasicStates;

public class LiftBasic extends CommandBase {
  /** Creates a new LiftBasic. */
  private final Lift lifter = Robot.lift;
  private LiftBasicStates state;
  private final double power;
  private boolean liftUp;
  private boolean liftdown;
  public LiftBasic(LiftBasicStates state, double power) {
    this.state = state;
    this.power = power;
    addRequirements(lifter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    liftUp = lifter.highLimit();
    liftdown = lifter.lowLimit();
    switch(state)
    {
      //-expl Set power to be positive until top limit switch is pressed
      case UP:
      if(liftUp)
      {
       lifter.setPower(power);
      }
      else
      {
        lifter.setPower(0);
      }
      break;

      //-expl Set power to be negative until bottom limit switch is pressed
      case DOWN:
      if(liftdown)
      {
      lifter.setPower(-power);
      }
      else
      {
        //lifter.setPower(0);
      }
      break;

      //-expl Set power to be 0
      case STOP:
      if(lifter.V2holder != 0 && lifter.V2holder > 5)
      {
        lifter.setSmartPos(lifter.V2holder);
      }
      else
      {
        lifter.setPower(0);
      }

      /* 
      if(!lifter.lowLimit() && Math.abs(lifter.getPos()) > 1)
      {
        lifter.setPos(0);
      }
      */
      
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
