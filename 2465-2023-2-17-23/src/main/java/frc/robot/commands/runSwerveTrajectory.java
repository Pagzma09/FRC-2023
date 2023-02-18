// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*Written by MBJ */

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

/* */
public class runSwerveTrajectory extends CommandBase {
  /** Creates a new runSwerveTrajectory. */
  private Drive drive = Robot.drive;
  private final SwerveControllerCommand controller;
  private final Trajectory path;
  //private final TrajectoryConfig config;
  private final PIDController xController;
  private final PIDController yController;
  private boolean isDone = false;


  public runSwerveTrajectory(Trajectory path) {

    this.path = path;
    
    var thetacontroller = new ProfiledPIDController(AutoConstants.kPThetaController, 
    0, 0,
    AutoConstants.aThetaControllerConstraints);

    thetacontroller.enableContinuousInput(-Math.PI, Math.PI);

    xController = new PIDController(AutoConstants.kPXController, 0, 0);
    yController = new PIDController(AutoConstants.kPYController, 0, 0);

    controller = new SwerveControllerCommand(
    path, 
    drive::getPose, 
    Constants.kinematics, 
    xController, 
    yController,
    thetacontroller,
    drive::setModuleStates, 
    drive);

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    drive.resetOdometry(path.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(!controller.isFinished())
    {
    controller.execute();
    }

    isDone = true;
    isFinished();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive
      (0, 
      0.05, 
      0, 
      false);

    Constants.frontRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.frontLeftDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearRightDrive.setIdleMode(IdleMode.kBrake);
    Constants.rearLeftDrive.setIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isDone)
    {
    return true;
    }
    else
    {
      return false;
    }
  }
}
