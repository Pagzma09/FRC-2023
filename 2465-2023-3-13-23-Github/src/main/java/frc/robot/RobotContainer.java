// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.CAEBasic;
import frc.robot.commands.CAEOperator;
import frc.robot.commands.ClawBasic;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtensionBasic;
import frc.robot.commands.GoToExtensionEncoder;
import frc.robot.commands.GoToExtensionEncoderNK;
import frc.robot.commands.GoToLift;
import frc.robot.commands.GoToLiftEncoder;
import frc.robot.commands.GoToLiftEncoderV2;
import frc.robot.commands.InstantClawRotate;
import frc.robot.commands.LiftBasic;
import frc.robot.commands.LightController;
import frc.robot.commands.LightsBasic;
import frc.robot.commands.LightsV2_ChargedUpSpecific;
import frc.robot.commands.LimelightSetPivot;
import frc.robot.commands.LightsV2_Commands;
import frc.robot.commands.StickDrive;
import frc.robot.commands.WAEBasic;
import frc.robot.commands.WAEOperator;
import frc.robot.commands.WristBasic;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawAbsoluteEncoder;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.LightsV2;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.WristAbsoluteEncoder;
import frc.robot.subsystems.Claw.ClawBasicStates;
import frc.robot.subsystems.Extension.ExtensionBasicStates;
import frc.robot.subsystems.Lift.GoToLiftStates;
import frc.robot.subsystems.Lift.LiftBasicStates;
import frc.robot.subsystems.Lights.Light_Controller_States;
import frc.robot.subsystems.Wrist.WristBasicStates;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 * 
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
  private final WristAbsoluteEncoder waer = Robot.wae;
  private final Lights lighter = Robot.lights;
  private final Limelight limelighter = Robot.limelight;
  public static LightsV2 lightsvtwoer = new LightsV2();

  // instaniating subysstems here per instructions at
  // https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html
  // (Note ExampleSubsystem above also instanciated here)

  // ? private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();?


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final Joystick controller = new Joystick(1);    
  private static final Joystick buttonboard = new Joystick(2);
  //private static final Joystick experimental = new Joystick(3);
  private static final Joystick DreamJoystick = new Joystick(5);

  private static final Joystick buttonBoardV2 = new Joystick(4);
  private static final Joystick buttonBoardClawStick = new Joystick(3);

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
  private static final JoystickButton clawOpenBuildDelayed = new JoystickButton(buttonboard, 11);
  private static final JoystickButton clawCloseBuildDelayed = new JoystickButton(buttonboard, 12);

  private static final JoystickButton b3 = new JoystickButton(DreamJoystick, 3);
  private static final JoystickButton b4 = new JoystickButton(DreamJoystick, 4);
  private static final JoystickButton b5 = new JoystickButton(DreamJoystick, 5);
  private static final JoystickButton b6 = new JoystickButton(DreamJoystick, 6);
  private static final JoystickButton b7 = new JoystickButton(DreamJoystick, 7);
  private static final JoystickButton b10 = new JoystickButton(DreamJoystick, 10);

  private static final JoystickButton leftcross_upper_white_button = new JoystickButton(buttonBoardV2, 9);
  private static final JoystickButton lefftcross_lower_white_button = new JoystickButton(buttonBoardV2, 12);
  private static final JoystickButton leftcross_left_yellow_button = new JoystickButton(buttonBoardV2, 8);
  private static final JoystickButton leftcross_right_yellow_button = new JoystickButton(buttonBoardV2, 3);
  private static final JoystickButton leftcross_left_green_button = new JoystickButton(buttonBoardV2, 10);
  private static final JoystickButton leftcross_right_green_button = new JoystickButton(buttonBoardV2, 2);

  private static final JoystickButton left_blue_button = new JoystickButton(buttonBoardV2, 11);
  private static final JoystickButton right_blue_button = new JoystickButton(buttonBoardV2, 5);

  private static final JoystickButton rightcross_upper_white_button = new JoystickButton(buttonBoardV2, 1);
  private static final JoystickButton rightcross_lower_white_button = new JoystickButton(buttonBoardV2, 4);
  private static final JoystickButton rightcross_left_yellow_button = new JoystickButton(buttonBoardV2, 6);
  private static final JoystickButton rightcross_right_yellow_button = new JoystickButton(buttonBoardV2, 14);
  private static final JoystickButton rightcross_left_green_button = new JoystickButton(buttonBoardV2, 7);
  private static final JoystickButton rightcross_right_red_button = new JoystickButton(buttonBoardV2, 13);

  private static final JoystickButton c1 = new JoystickButton(buttonBoardClawStick, 1);
  private static final JoystickButton c2 = new JoystickButton(buttonBoardClawStick, 2);
  private static final JoystickButton c3 = new JoystickButton(buttonBoardClawStick, 3);
  private static final JoystickButton c5 = new JoystickButton(buttonBoardClawStick, 5);
  private static final JoystickButton c6 = new JoystickButton(buttonBoardClawStick, 6);
  private static final POVButton c0 = new POVButton(buttonBoardClawStick, 0);
  private static final POVButton c180 = new POVButton(buttonBoardClawStick, 180);

  private static final JoystickButton d9 = new JoystickButton(controller, 9);
  private static final JoystickButton d10 = new JoystickButton(controller, 10);
  private final JoystickButton yButton = new JoystickButton(controller, 4);
  private final JoystickButton aButton = new JoystickButton(controller, 1);
  private final JoystickButton bButton = new JoystickButton(controller, 2);
  
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
    waer.setDefaultCommand(new WAEBasic());
    lighter.setDefaultCommand(new LightsBasic());
    //limelighter.setDefaultCommand(new LimelightSetPivot(limelighter.limepivpos));
    
    LightsV2_Commands.ShowBatteryState(lightsvtwoer).schedule();
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
    extensionOut.whileTrue(new ExtensionBasic(ExtensionBasicStates.OUT, 0.3));
    extensionIn.whileTrue(new ExtensionBasic(ExtensionBasicStates.IN, 0.32));
    clawSuck.whileTrue(new ClawBasic(ClawBasicStates.Suck, 0.3));
    clawSpit.whileTrue(new ClawBasic(ClawBasicStates.Spit, 0.3));
   // clawOpen.whileTrue(new InstantClawRotate(5));
   // clawClose.whileTrue(new InstantClawRotate(-2));
    clawOpen.whileTrue(new CAEOperator(0.2));
    clawClose.whileTrue(new CAEOperator(0.02));
    wristOut.whileTrue(new WristBasic(WristBasicStates.OUT, 1));
    wristIn.whileTrue(new WristBasic(WristBasicStates.IN, 1));
    clawOpenBuildDelayed.toggleOnTrue(new GoToLiftEncoder(GoToLiftStates.HIGH));
    clawCloseBuildDelayed.whileTrue(new GoToLiftEncoder(GoToLiftStates.LOW));
    //AutoBalanceButton.toggleOnTrue(new AutoBalanceV2());

    /* 
    //MIGRATED
    liftgotoLowestAuto.toggleOnTrue(new GoToLift(GoToLiftStates.LOW, 0.2));
    liftgotoMiddleAuto.toggleOnTrue(new GoToLift(GoToLiftStates.MIDDLE, 0.2));
    liftgotoHighAuto.toggleOnTrue(new GoToLift(GoToLiftStates.HIGH, 0.2));
    liftgoToHighAutoEncoder.toggleOnTrue(new GoToLiftEncoderV2(285));
    liftgoToMidAutoEncoder.toggleOnTrue(new GoToLiftEncoderV2(160));
    liftgoToLowAutoEncoder.toggleOnTrue(new GoToLiftEncoderV2(10));

    //NOT TESTED
    extendgoIn.toggleOnTrue(new GoToExtensionEncoder(0));
    extendgoOut.toggleOnTrue(new GoToExtensionEncoder(250));
    extendandliftupandout.toggleOnTrue(new ComplexExtendAndLift(250, 285));
    extendandliftdownandstow.toggleOnTrue(new ComplexExtendAndLift(10, 10));

    //MY MAPPINGS
    */
    b3.toggleOnTrue(new CAEOperator(0.15));
    b4.toggleOnTrue(new CAEOperator(0.02));
    b10.toggleOnTrue(new CAEOperator(0.1));
    b5.whileTrue(new ClawBasic(ClawBasicStates.Suck, 0.3));
    b6.whileTrue(new ClawBasic(ClawBasicStates.Spit, 0.3));
    b7.toggleOnTrue(new WAEOperator(0.03));

    //Actual Button Board Mappings
    leftcross_upper_white_button.whileTrue(new LiftBasic(LiftBasicStates.UP, 0.6));
    lefftcross_lower_white_button.whileTrue(new LiftBasic(LiftBasicStates.DOWN, 0.6));
    leftcross_left_yellow_button.whileTrue(new ExtensionBasic(ExtensionBasicStates.OUT, 0.6));
    leftcross_right_yellow_button.whileTrue(new ExtensionBasic(ExtensionBasicStates.IN, 0.6));
    leftcross_left_green_button.toggleOnTrue(new WAEOperator(0.37));
    leftcross_right_green_button.toggleOnTrue(new WAEOperator(0.04));

    left_blue_button.toggleOnTrue(new WAEOperator(0.23));
    right_blue_button.toggleOnTrue(new WAEOperator(0.3));

    //rightcross_upper_white_button.toggleOnTrue(new ComplexLiftExtendWristCommand(260,  150, 0.04).andThen(new WAEOperator(0.23)));
    //rightcross_lower_white_button.toggleOnTrue(new ComplexLiftExtendWristCommand(7, -3-150, 0.04).beforeStarting(new WAEOperator(0.03)));
    rightcross_upper_white_button.toggleOnTrue(new GoToLiftEncoderV2(82).alongWith(new GoToExtensionEncoderNK(37)));
    rightcross_lower_white_button.toggleOnTrue(new GoToLiftEncoderV2(0).alongWith(new GoToExtensionEncoderNK(4)).alongWith(new WAEOperator(0.03)));
    //rightcross_right_yellow_button.toggleOnTrue(new ComplexLiftExtendWristCommand(110, 210-150, 0.04).andThen(new WAEOperator(0.23)));
    rightcross_right_yellow_button.toggleOnTrue(new GoToLiftEncoderV2(60).alongWith(new GoToExtensionEncoderNK(-1)));
    //rightcross_left_yellow_button.toggleOnTrue(new ComplexLiftExtendWristCommand(39, 12-150, 0.08));//39,12,0.08
    rightcross_left_yellow_button.toggleOnTrue(new GoToLiftEncoderV2(70).alongWith(new GoToExtensionEncoderNK(60)));
    rightcross_left_green_button.toggleOnTrue(new CAEOperator(0.03));
    rightcross_right_red_button.toggleOnTrue(new CAEOperator(0.15));

    c1.toggleOnTrue(new CAEOperator(0.15));
    c2.toggleOnTrue(new CAEOperator(0.13));
    c3.toggleOnTrue(new CAEOperator(0.02));
    c5.whileTrue(new ClawBasic(ClawBasicStates.Suck, 0.15));
    c6.whileTrue(new ClawBasic(ClawBasicStates.Spit, 0.15));
    c0.whileTrue(new ClawBasic(ClawBasicStates.Spit, 0.15));
    c180.whileTrue(new ClawBasic(ClawBasicStates.Suck, 0.15));

    d9.toggleOnTrue(new LightController(Light_Controller_States.ADD));
    d10.toggleOnTrue(new LightController(Light_Controller_States.SUBTRACT));
    d9.onTrue(LightsV2_ChargedUpSpecific.requestCube(lightsvtwoer));
    d10.onTrue(LightsV2_ChargedUpSpecific.requestCone(lightsvtwoer));
    yButton.toggleOnTrue(new LimelightSetPivot(0));
    aButton.toggleOnTrue(new LimelightSetPivot(0.25));
    bButton.toggleOnTrue(new LimelightSetPivot(0.125));
    
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

  //This function exists as an interface between the Imperative (code in Robot.java), and the Declarative (code everywhere else).
  public static void showAllianceColor() {
    LightsV2_Commands.showAllianceColour(lightsvtwoer).schedule();
  }

}
