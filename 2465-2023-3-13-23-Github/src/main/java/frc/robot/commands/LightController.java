// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.Light_Controller_States;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LightController extends InstantCommand {
  private final Lights lighter = Robot.lights;
  Light_Controller_States states;
  public LightController(Light_Controller_States states) {
    this.states = states;
    addRequirements(lighter);
    // Use addRequirements() her to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(states)
    {
      case ADD:
      lighter.light_controller = lighter.light_controller + 1;

      if(lighter.light_controller > 7)
      {
        lighter.light_controller = 0;
      }

      break;
      case SUBTRACT:
      lighter.light_controller = lighter.light_controller - 1;

      if(lighter.light_controller < 0)
      {
        lighter.light_controller = 7;
      }
      break;
      
      default:
      lighter.light_controller = lighter.light_controller;
      break;
    }
  }
}
