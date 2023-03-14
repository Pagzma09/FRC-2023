// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private final DigitalOutput P5 = Constants.p5;
  private final DigitalOutput P6 = Constants.p6;
  private final DigitalOutput P7 = Constants.p7;
  public int light_controller;
  public enum Light_Controller_States {ADD, SUBTRACT};
  public Lights() {
    light_controller = 5;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SmartDashValues()
  {
    SmartDashboard.putNumber("Light Index", light_controller);
  }

  public void setLights()
  {
    switch(light_controller)
    {
      case 7:
      P5.set(false);
      P6.set(false);
      P7.set(false);
      break;
      case 6:
      P5.set(true);
      P6.set(false);
      P7.set(false);
      break;
      case 5:
      P5.set(false);
      P6.set(true);
      P7.set(false);
      break;
      case 4:
      P5.set(true);
      P6.set(true);
      P7.set(false);
      break;
      case 3:
      P5.set(false);
      P6.set(false);
      P7.set(true);
      break;
      case 2:
      P5.set(true);
      P6.set(false);
      P7.set(true);
      break;
      case 1:
      P5.set(false);
      P6.set(true);
      P7.set(true);
      break;
      case 0:
      P5.set(true);
      P6.set(true);
      P7.set(true);
      break;
      default:
      P5.set(true);
      P6.set(true);
      P7.set(true);
      break;
    }
  }
}
