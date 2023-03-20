// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;
import java.util.random.RandomGenerator;

import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.commands.AutoBalanceV2;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawAbsoluteEncoder;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristAbsoluteEncoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  public static Drive drive;
  public static Lift lift;
  public static Extension extension;
  public static Claw claw;
  public static Wrist wrist;
  public static ClawAbsoluteEncoder cae;
  public static WristAbsoluteEncoder wae;
  public static Limelight limelight;
  public static Lights lights;
  private SendableChooser<Command> autoChooser;
  private Random justforfunnzez;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    System.out.println("Hello world!");
    Constants.maininit();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    drive = new Drive();
    lift = new Lift();
    extension = new Extension();
    claw = new Claw();
    cae = new ClawAbsoluteEncoder();
    wrist = new Wrist();
    wae = new WristAbsoluteEncoder();
    limelight = new Limelight();
    lights = new Lights();
    m_robotContainer = new RobotContainer();
    autoChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    drive.zeroHeading();

    justforfunnzez = new Random();
    lights.light_controller = 5;
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    drive.SmartDashValues();
    lift.SmartDashValues();
    extension.SmartDashValues();
    claw.SmartDashValues();
    wrist.SmartDashValues();
    lights.SmartDashValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    int boundsforfun = 7;

    int sonicsays = justforfunnzez.nextInt(boundsforfun);

    if(sonicsays == 7)
    {
      sonicsays = 6;
    }

    lights.light_controller = 5;
    
  }

  @Override
  public void disabledPeriodic() {
    claw.clawDesiredPosition = claw.getPosition();
    wrist.desired_wrist_pos = wrist.getPosition();

    if(lift.V2holder != 0)
    {
      lift.V2holder = lift.getPos();
    }
    
    if(extension.holder != 0)
    {
      extension.holder = extension.getPosition();
    }
    //cae.desired_position = cae.getPosition();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    RobotContainer.showAllianceColor();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    else
    {
      drive.resetOdometry(new Pose2d(0,0, new Rotation2d(Math.toRadians(0))));
    }

    limelight.setPos(0);
    RobotContainer.prepEndgameWarning();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
