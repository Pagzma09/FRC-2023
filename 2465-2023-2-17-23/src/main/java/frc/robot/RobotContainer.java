// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AddressLEDs;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autos;
import frc.robot.commands.CAEBasic;
import frc.robot.commands.CAEOperator;
import frc.robot.commands.ClawBasic;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtensionBasic;
import frc.robot.commands.GoToExtensionEncoder;
import frc.robot.commands.GoToLift;
import frc.robot.commands.GoToLiftEncoder;
import frc.robot.commands.InstantClawRotate;
import frc.robot.commands.LiftBasic;
import frc.robot.commands.StickDrive;
import frc.robot.commands.WristBasic;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawAbsoluteEncoder;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.AddressableLEDInterface.LEDCommands;
import frc.robot.subsystems.Claw.ClawBasicStates;
import frc.robot.subsystems.Extension.ExtensionBasicStates;
import frc.robot.subsystems.Lift.GoToLiftStates;
import frc.robot.subsystems.Lift.LiftBasicStates;
import frc.robot.subsystems.Wrist.WristBasicStates;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drive driver = Robot.drive;
  private final Lift lifter = Robot.lift;
  private final Extension extensioner = Robot.extension;
  private final Claw clawer = Robot.claw;
  private final Wrist wrister = Robot.wrist;
  private final ClawAbsoluteEncoder caer = Robot.cae;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final Joystick controller = new Joystick(1);    
  private static final Joystick buttonboard = new Joystick(2);
  private static final Joystick experimental = new Joystick(3);
  private static final JoystickButton liftUp = new JoystickButton(buttonboard, 1);
  private static final JoystickButton liftDown = new JoystickButton(buttonboard, 2);
  private static final JoystickButton extensionOut = new JoystickButton(buttonboard, 3);
  private static final JoystickButton extensionIn = new JoystickButton(buttonboard, 4);
  private static final JoystickButton clawSuck = new JoystickButton(buttonboard, 5);
  private static final JoystickButton clawSpit = new JoystickButton(buttonboard, 6);
  private static final JoystickButton clawOpen = new JoystickButton(buttonboard, 7);
  private static final JoystickButton clawClose = new JoystickButton(buttonboard, 8);
  private static final JoystickButton wristOut = new JoystickButton(buttonboard, 9);
  private static final JoystickButton wristIn = new JoystickButton(buttonboard, 10);
  private static final JoystickButton toggleLEDsFwd = new JoystickButton(buttonboard, 11);
  private static final JoystickButton toggleLEDsBck = new JoystickButton(buttonboard, 12);
  private static final JoystickButton clawOpenBuildDelayed = new JoystickButton(buttonboard, 11);
  private static final JoystickButton clawCloseBuildDelayed = new JoystickButton(buttonboard, 12);
  private static final JoystickButton AutoBalanceButton = new JoystickButton(experimental, 1);
  private static final JoystickButton liftgotoLowestAuto = new JoystickButton(experimental, 2);
  private static final JoystickButton liftgotoMiddleAuto = new JoystickButton(experimental, 3);
  private static final JoystickButton liftgotoHighAuto = new JoystickButton(experimental, 4);
  private static final JoystickButton liftgoToHighAutoEncoder = new JoystickButton(experimental, 5);
  private static final JoystickButton liftgoToMidAutoEncoder = new JoystickButton(experimental, 6);
  private static final JoystickButton liftgoToLowAutoEncoder = new JoystickButton(experimental, 7);
  private static final JoystickButton extendgoOut = new JoystickButton(experimental, 8);
  private static final JoystickButton extendgoIn = new JoystickButton(experimental, 9);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    driver.setDefaultCommand(new StickDrive());
    lifter.setDefaultCommand(new LiftBasic(LiftBasicStates.STOP, 0));
    extensioner.setDefaultCommand(new ExtensionBasic(ExtensionBasicStates.STOP, 0));
    clawer.setDefaultCommand(new ClawBasic(ClawBasicStates.Stop, 0));
    wrister.setDefaultCommand(new WristBasic(WristBasicStates.HOLD, 0));
    caer.setDefaultCommand(new CAEBasic());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    liftUp.whileTrue(new LiftBasic(LiftBasicStates.UP, 0.6));
    liftDown.whileTrue(new LiftBasic(LiftBasicStates.DOWN, 0.6));
    extensionOut.whileTrue(new ExtensionBasic(ExtensionBasicStates.OUT, 0.6));
    extensionIn.whileTrue(new ExtensionBasic(ExtensionBasicStates.IN, 0.6));
    clawSuck.whileTrue(new ClawBasic(ClawBasicStates.Suck, 0.3));
    clawSpit.whileTrue(new ClawBasic(ClawBasicStates.Spit, 0.3));
    //clawOpen.whileTrue(new InstantClawRotate(5));
    //clawClose.whileTrue(new InstantClawRotate(-2));
    clawOpen.whileTrue(new CAEOperator(0.1));
    clawClose.whileTrue(new CAEOperator(0.97));
    wristOut.whileTrue(new WristBasic(WristBasicStates.OUT, 1));
    wristIn.whileTrue(new WristBasic(WristBasicStates.IN, 1));
    clawOpenBuildDelayed.toggleOnTrue(new GoToLiftEncoder(GoToLiftStates.HIGH));
    clawCloseBuildDelayed.whileTrue(new GoToLiftEncoder(GoToLiftStates.LOW));
    AutoBalanceButton.toggleOnTrue(new AutoBalance());
    toggleLEDsBck.onTrue(new AddressLEDs(3, 5, LEDCommands.CycleBack));
    toggleLEDsFwd.onTrue(new AddressLEDs(3, 5, LEDCommands.CycleFwd));

    //MIGRATED
    liftgotoLowestAuto.toggleOnTrue(new GoToLift(GoToLiftStates.LOW, 0.2));
    liftgotoMiddleAuto.toggleOnTrue(new GoToLift(GoToLiftStates.MIDDLE, 0.2));
    liftgotoHighAuto.toggleOnTrue(new GoToLift(GoToLiftStates.HIGH, 0.2));
    liftgoToHighAutoEncoder.toggleOnTrue(new GoToLiftEncoder(GoToLiftStates.HIGH));
    liftgoToMidAutoEncoder.toggleOnTrue(new GoToLiftEncoder(GoToLiftStates.MIDDLE));
    liftgoToLowAutoEncoder.toggleOnTrue(new GoToLiftEncoder(GoToLiftStates.LOW));

    //NOT TESTED
    extendgoIn.whileTrue(new GoToExtensionEncoder(0));
    extendgoOut.whileTrue(new GoToExtensionEncoder(100));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
