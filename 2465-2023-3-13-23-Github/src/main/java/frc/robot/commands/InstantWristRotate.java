// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InstantWristRotate extends InstantCommand {
  private final Wrist wrister = Robot.wrist;
  private final double positiontogoto;
  public InstantWristRotate(double position) {
    positiontogoto = position;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  //-expl See InstantClawRotate.java. Like that, but for wrist rotation.
  public void initialize() {
    wrister.goToPosition(positiontogoto);
  }
}
