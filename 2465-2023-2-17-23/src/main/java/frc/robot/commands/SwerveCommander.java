// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;

/** Add your docs here. */
public class SwerveCommander {
  private final Drive driver = Robot.drive;

  public SwerveCommander()
  {

  }

  public Command RunTrajectory(Trajectory traj)
  {

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.aThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            traj,
            driver::getPose, // Functional interface to feed supplier
            Constants.kinematics,
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driver::setModuleStates,
            driver);

    // Reset odometry to the starting pose of the trajectory.
    driver.resetOdometry(traj.getInitialPose());
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> driver.Stop());
  }
}
