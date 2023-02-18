// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Extension;

public class GoToExtensionEncoder extends CommandBase {
  /** Creates a new GoToExtensionEncoder. */
  private final Extension extender = Robot.extension;
  private final double pos;
  public GoToExtensionEncoder(double pos) {
    this.pos = pos;
    addRequirements(extender);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //extender.extension_enc_troubleshooter = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extender.setPosition(pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    extender.extension_enc_troubleshooter = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
      return false;
  }
}
