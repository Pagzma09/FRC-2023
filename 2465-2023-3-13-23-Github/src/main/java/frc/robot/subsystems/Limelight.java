// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final NetworkTable limelight = Constants.limelight;
  private final NetworkTableEntry camMode = limelight.getEntry("camMode");
  private final Servo limelightpivoter = Constants.limelightpivot;
  public double limepivpos = 0;
  private double mode = 0;
  public Limelight() {
    limelight.getEntry("camMode").setValue(1);
  }

  @Override
  public void periodic() {
    mode = camMode.getValue().getDouble();

    SmartDashboard.putNumber("Limelight Mode", mode);
    SmartDashboard.putNumber("Limelight Pivot Pos", limelightpivoter.get());
    SmartDashboard.putNumber("Limelight Set Pos", limepivpos);
    //limelight.getEntry("camMode").setValue(1);
    // This method will be called once per scheduler run
  }

  public void setPos(double pos)
  {
    limepivpos = pos;
    limelightpivoter.set(pos);
  }
}
