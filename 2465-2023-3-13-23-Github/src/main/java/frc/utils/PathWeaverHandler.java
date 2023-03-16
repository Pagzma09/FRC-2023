// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.AutoConstants;

/** Add your docs here. */
public class PathWeaverHandler {

    public Trajectory generateTrajectories(String trajectory_name)
    {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), 
            List.of(new Translation2d(0.25, 0)), 
            new Pose2d(0.5, 0, new Rotation2d(Math.toRadians(0))), 
            AutoConstants.config);
            
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory_name);
            traj = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectory_name, ex.getStackTrace());
         }
        
        return traj; 
    }
}
