// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristBasicStates;

public class WristBasic extends CommandBase {
  /** Creates a new WristBasicState. */
  private final Wrist wrister = Robot.wrist;
  private final WristBasicStates state;
  private final double enacted_input;
  public WristBasic(WristBasicStates state, double enacted_input) {
    this.state = state;
    this.enacted_input = enacted_input;
    addRequirements(wrister);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state)
    {
      //-expl Set wrist to desired position

      case OUT:
      wrister.setPos(enacted_input);
      break;

      case IN:
      wrister.setPos(-enacted_input);
      break;

      case HOLD:
      wrister.hold();
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
