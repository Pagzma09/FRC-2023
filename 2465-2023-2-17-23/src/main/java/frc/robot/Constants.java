// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.System.Logger.Level;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  //DRIVE 
  public static CANSparkMax frontRightDrive;
  public static CANSparkMax frontRightRotate;
  public static CANSparkMax frontLeftDrive;
  public static CANSparkMax frontLeftRotate;
  public static CANSparkMax rearRightDrive;
  public static CANSparkMax rearRightRotate;
  public static CANSparkMax rearLeftDrive;
  public static CANSparkMax rearLeftRotate;
  public static SwerveDriveKinematics kinematics;
  public static double controllerDeadZone;

  public static AHRS imu;

  //LIFT
  public static CANSparkMax lift;
  public static DigitalInput Level1;
  public static DigitalInput Level2;
  public static DigitalInput Level3;

  //EXTENSION
  public static CANSparkMax extension;
  public static DigitalInput forwardlimit;
  public static DigitalInput reverselimit;

  //WRIST
  public static CANSparkMax wrist;

  //CLAW
  public static CANSparkMax ClawRotate;
  public static TalonFX ClawSpin1;
  public static TalonFX ClawSpin2;
  

  public static void maininit()
  {
    driveinit();

    liftinit();

    extensioninit();

    wristinit();

    clawinit();
  }

  static void driveinit()
  {
    imu = new AHRS();

    frontLeftDrive = new CANSparkMax(1, MotorType.kBrushless);
    frontLeftDrive.setInverted(false);
    frontLeftDrive.setIdleMode(IdleMode.kCoast);
    frontLeftDrive.setInverted(true);

    frontLeftRotate = new CANSparkMax(2, MotorType.kBrushless);
    frontLeftRotate.setInverted(false);
    frontLeftRotate.setIdleMode(IdleMode.kCoast);
    frontLeftRotate.setInverted(true);

     
    rearLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
    rearLeftDrive.setInverted(false);
    rearLeftDrive.setIdleMode(IdleMode.kCoast);
   // rearLeftDrive.setInverted(true);

    rearLeftRotate = new CANSparkMax(4, MotorType.kBrushless);
    rearLeftRotate.setInverted(false);
    rearLeftRotate.setIdleMode(IdleMode.kCoast);
    rearLeftRotate.setInverted(true);
    

    frontRightDrive = new CANSparkMax(5, MotorType.kBrushless);
    frontRightDrive.setInverted(false);
    frontRightDrive.setIdleMode(IdleMode.kCoast);
    frontRightDrive.setInverted(true);

    frontRightRotate = new CANSparkMax(6, MotorType.kBrushless);
    frontRightRotate.setInverted(false);
    frontRightRotate.setIdleMode(IdleMode.kCoast);
    frontRightRotate.setInverted(true);

    rearRightDrive = new CANSparkMax(7, MotorType.kBrushless);
    rearRightDrive.setInverted(false);
    rearRightDrive.setIdleMode(IdleMode.kCoast);
    rearRightDrive.setInverted(true);

    rearRightRotate = new CANSparkMax(8, MotorType.kBrushless);
    rearRightRotate.setInverted(false);
    rearRightRotate.setIdleMode(IdleMode.kCoast);
    rearRightRotate.setInverted(true);

    final double TrackWidth = Units.inchesToMeters(22.5); //Measure before using; L - R
    final double WheelBase = Units.inchesToMeters(22.5); //Measure before using; F - B

    Translation2d frontLeftModuleLocation = new Translation2d(WheelBase/2, TrackWidth/2);
    Translation2d frontRightModuleLocation = new Translation2d(WheelBase/2, -(TrackWidth/2));
    Translation2d rearLeftModuleLocation = new Translation2d(-(WheelBase/2), TrackWidth/2);
    Translation2d rearRightModuleLocation = new Translation2d(-(WheelBase/2), -(TrackWidth/2));

   kinematics = new SwerveDriveKinematics(
     frontLeftModuleLocation, frontRightModuleLocation, rearLeftModuleLocation, rearRightModuleLocation
   );

   controllerDeadZone = 0.05;



  }

  public static final class DriveConstants
  {
    public static final double ksVolts = 0.23711;
    public static final double kvVoltSecondsPerMeter = 2.2243;
    public static final double kaVoltSecondsSquaredPerMeter = 0.49753;
    public static final double kPDriveVel = 3.1331;

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(3);
    public static final double kMaxAngularSpeed = Math.PI;
  }

  public static final class ModuleConstants
  {
  //public static final int DrivingMotorPinionTeeth = 12;

  //public static final turningMotorInverted = true;

  public static final double DriveEncoderCPR = 42;
  public static final double DriveEncoderTrueCPR = 5.5;
  public static final double WheelDiametersMeters = Units.inchesToMeters(4);
  public static final double WheelCircumferenceMeters = WheelDiametersMeters * Math.PI;
  public static final double DriveEncoderDistancePerPulse = (WheelCircumferenceMeters) / (DriveEncoderTrueCPR);
  public static final double DriveRPSec = ((5676/60)*WheelCircumferenceMeters)/5.5;

  public static final double TurningEncoderDistancePerpulse = 2 * Math.PI;

  public static final double kDriveP = 0.5;
  public static final double kDriveI = 0;
  public static final double kDriveD = 0;
  public static final double kDriveFF = 1/DriveRPSec;
  public static final double kMinDriveOutput = -0.85;
  public static final double kMaxDriveOutput = 0.85;

  public static final double kTurnP = 0.5;
  public static final double kTurnI = 0;
  public static final double kTurnD = 0;
  public static final double kTurnFF = 0;
  public static final double kMinTurnOutput = -0.8;
  public static final double kMaxturnOutput = 0.8;




 }

 public static class AutoConstants
 {
   public static final double aMaxMetersPerSecond = Units.feetToMeters(11);
   public static final double aMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(5);
   public static final double aMaxAngularSpeedRadiansPerSecond = Math.PI*2;
   public static final double aMaxAngularAccelRadiansPerSecondSquared = Math.PI;

   public static final double kPXController = DriveConstants.kPDriveVel;
   public static final double kPYController = DriveConstants.kPDriveVel;
   public static final double kPThetaController = 1;

   public static final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.aMaxMetersPerSecond, AutoConstants.aMaxAccelerationMetersPerSecondSquared);

   public static final TrapezoidProfile.Constraints aThetaControllerConstraints = new TrapezoidProfile.Constraints(
    aMaxAngularSpeedRadiansPerSecond, aMaxAccelerationMetersPerSecondSquared);
      
 }

 static void liftinit()
 {
  lift = new CANSparkMax(9, MotorType.kBrushless);
  lift.set(0);

  Level1 = new DigitalInput(0);
  Level2 = new DigitalInput(1);
  Level3 = new DigitalInput(2);
 }

 public static final class liftConstants
 {
  public static final double liftP = 0.2;
  public static final double liftI = 0;
  public static final double liftD = 0;
  public static final double liftMinOut = -0.8;
  public static final double liftMaxOut = 0.8;
 }

 static void extensioninit()
 {
  extension = new CANSparkMax(10, MotorType.kBrushless);
  extension.setInverted(true);
  extension.set(0);

  forwardlimit = new DigitalInput(4);
  reverselimit = new DigitalInput(5);
 }

 public static class ExtensionConstants
 {
  public static final double extensionP = 0.2;
  public static final double extensionI = 0;
  public static final double extensionD = 0;
  public static final double extensionMinOut = -0.6;
  public static final double extenionMaxIn = -0.6;
 }

 static void wristinit()
 {
  wrist = new CANSparkMax(11, MotorType.kBrushless);
  wrist.setInverted(true);
  wrist.set(0);
 }

 public static class wristConstants
 {
   public static final double wristP = 0.3;
   public static final double wristI = 0.0;
   public static final double wristD = 0.0;
   public static final double wristMinOut = -0.4;
   public static final double wristMaxOut = 0.4;
 }

 static void clawinit()
 {
   ClawRotate = new CANSparkMax(12, MotorType.kBrushless);
   //ClawRotate.setInverted(true);
   ClawRotate.set(0);

   ClawSpin1 = new TalonFX(1);
   //ClawSpin1.set(0);

   ClawSpin2 = new TalonFX(2);
   ClawSpin2.setInverted(true);
   //ClawSpin2.set(0);
 }

 public static class clawConstants
 {
   public static final double clawRotateP = 0.3;
   public static final double clawRotateI = 0;
   public static final double clawRotateD = 0;
   public static final double clawMinOut = -0.5;
   public static final double clawMaxOut = 0.5;
 }

 public static class LEDChannels
 {
    public static final int startChannel = 5;
    public static final int numChannels = 3;
 }
 /*
  * TODO:
    - Finish Assigning CAN ID's for TalonFx's
    - Finish Claw rudimentary code
    - Get Grabber working
    - Calibrate wrist encoder
    - SysID
    - Test Auto Code
    - Install HAL's for lift and extension
  */

}
