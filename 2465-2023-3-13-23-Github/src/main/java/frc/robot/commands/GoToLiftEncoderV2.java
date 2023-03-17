// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lift;

public class GoToLiftEncoderV2 extends CommandBase {
  /** Creates a new GoToLiftEncoderV2. */
  private final Lift lifter = Robot.lift;
  private final double pos;
  public GoToLiftEncoderV2(double pos) {
     this.pos = pos;
     addRequirements(lifter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //lifter.V2handler = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lifter.setSmartPos(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(lifter.returnPIDError(pos) < 5)
    {
      //lifter.V2handler = true;
      return true;
    }
    else
    {
    return false;
    }
  }
}
