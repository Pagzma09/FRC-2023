// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*Written by MBJ */

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;

public class StickDrive extends CommandBase {
  private final Drive swervy = Robot.drive;
  private final Joystick controller = RobotContainer.controller;
  /** Creates a new StickDrive. */
  public StickDrive() {
    addRequirements(swervy);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swervy.slewdrive(
      MathUtil.applyDeadband(-controller.getRawAxis(5), 0.05), 
      MathUtil.applyDeadband(-controller.getRawAxis(0)*.9, 0.05), 
      MathUtil.applyDeadband(-controller.getRawAxis(4)*.9, 0.05), 
    false,
    true);
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
