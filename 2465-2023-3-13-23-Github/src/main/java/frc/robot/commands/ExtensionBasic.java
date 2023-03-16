// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Extension.ExtensionBasicStates;

public class ExtensionBasic extends CommandBase {
  /** Creates a new ExtensionBasic. */

  //-expl Initialize extension, state, and power variables
  private final Extension extensioner = Robot.extension;
  private final ExtensionBasicStates state;
  private final double power;
  public ExtensionBasic(ExtensionBasicStates state, double power) {
    //-expl Set variables
    this.state = state;
    this.power = power;
    addRequirements(extensioner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //-expl Switch on state. Intutive.
    switch(state)
    {
      case OUT:
      extensioner.setPower(power);
      break;

      case IN:
      extensioner.setPower(-power);
      break;
      
      case STOP:
      extensioner.setPower(0);
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
