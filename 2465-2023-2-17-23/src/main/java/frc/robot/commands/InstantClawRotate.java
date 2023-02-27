// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantClawRotate extends InstantCommand {
  private final Claw clawer = Robot.claw;
  private final double positiontogo;
  public InstantClawRotate(double position) {
    positiontogo = position;
    addRequirements(clawer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  //-expl Short command to go to a selected position
  public void initialize() {
    clawer.goToRotate(-positiontogo);
  }
}
