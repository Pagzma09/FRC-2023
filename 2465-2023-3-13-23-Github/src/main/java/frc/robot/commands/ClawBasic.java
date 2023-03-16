// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Claw.ClawBasicStates;

public class ClawBasic extends CommandBase {
  /** Creates a new ClawBasic. */
  //-expl Initalize claw
  private final Claw clawer = Robot.claw;

  //-expl Create a new ClawBasicState
  private final ClawBasicStates state;
  
  //-expl Initalize a variable for the set power
  private final double enacted_input;
  
  public ClawBasic(ClawBasicStates state, double enacted_input) {
    
    //-expl Initalize variables    
    this.state = state;
    this.enacted_input = enacted_input;
    addRequirements(clawer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //-expl Set the speed of the claw (or the intake) to enacted_input or -enacted_input based on *state*
    switch(state)
    {
      case Open:
      clawer.setRotate(enacted_input);
      break;

      case Close:
      clawer.setRotate(-enacted_input);
      break;

      case Suck:
      clawer.setSpin(enacted_input);
      break;

      case Spit:
      clawer.setSpin(-enacted_input);
      break;
  
      case Stop:
      clawer.setSpin(0);
      //clawer.hold();
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
