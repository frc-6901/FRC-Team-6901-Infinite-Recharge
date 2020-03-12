/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.Util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    
  public static final class ControllerConstants {
      public static final int controllerPort = 0;
      public static final int controllerPort2 = 1;
  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 10;
    public static final double kIntakePower = -.45;
  }
  
  public static final class FeederConstants {
    public static final int kIndexerPort = 11;
    public static final int kAcceleratorWheelPort = 13;

    public static final double kIndexerSpeed = .5;
    public static final double kAcceleratorSpeed = -.32;
  }
  public static final class ClimberConstants {
    public static final int kClimberPort1 = 3;
    public static final int kClimberPort2 = 6;
    public static final int kClimberPort3 = 12;
    public static final int kClimberPort4 = 4;

    
    public static final int kUpstringPort = 8;

    public static final double kUpstringPower = .4;
    public static final double kClimberPower = .4;
  }

    
    //TODO Modify constants
  public static final class ShooterConstants{
      
      public static PolynomialRegression kPolynomial;
      
      public static double[][] kDistanceRPMTable = {
        {110, -3000},
        {135, -3110},
        {165, -3200},
        {195, -3550},
        {250, -4010},
        {220, -3700},
        {275, -4110},
        {300, -4250},
        {340, -4350},
        {373, -4450}
      };

      static {
        double[] x = new double[kDistanceRPMTable.length];
        double[] y = new double[kDistanceRPMTable.length];
        for (int i = 0; i < kDistanceRPMTable.length; i++) {
          x[i] = kDistanceRPMTable[i][0];
          y[i] = kDistanceRPMTable[i][1];
        }
        kPolynomial = new PolynomialRegression(x, y, 2);
      }
  
    //Ports
    public static final int kShooterIdBottom = 2;
    public static final int kShooterIdTop = 1;

    //PIDF
    public static final double kP = .00005;
    public static final double kF = .00017;

    // Simple Motor Feedforward
    public static final double kS = .114;
    public static final double kV = .126;
    public static final double kA = .012; // Acceleration seems negligible for this application


    // RPM Values
    public static final int kLongShotDiff = 750;
    public static final int kLongShotRPM = -5000;
    
    public static final int kRPMDifference = 1250; //1250
    public static final int kDefaultRPM = -2900;


  }

  public static final class LimelightConstants {
    public static final double kTargetHeight = 89.75;
    public static final double kLimelightHeight = 36.25;
    public static final double kLimelightAngle = 3;

    public static final double kXThreshold = 1.5;

  }
    //TODO Modify Constants and also change inner classes
    public static final class DriveConstants {

      public static final double kTurnModifier = .8; // Changes turn sensitivity
      public static final double kForwardModifier = .85; // Changes forward/backwards sensitivity

      //Limelight tuning
      public static final double kP = 0.07;
      public static final double kD = 0.3;

      public static final int kLeftMotor1Port = 2;
      public static final int kLeftMotor2Port = 2;
      public static final int kRightMotor1Port = 5;
      public static final int kRightMotor2Port = 1;
      public static final int kPigeonPort = 0;
  
    public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;
  
    public static final double kTrackwidthMeters = 0.5842;
    public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);
  
    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
          // Assumes the encoders are directly mounted on the wheel shafts
      (kWheelDiameterMeters * Math.PI) / ((double) kEncoderCPR);
  
    public static final boolean kGyroReversed = true;
  
    public static final int kMaxCurrent = 40;
      // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
      // These characterization values MUST be determined either experimentally or theoretically
      // for *your* robot's drive.
      // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
      // values for your robot.
    public static final double ksVolts = 1.1;
    public static final double kvVoltSecondsPerMeter = .28;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0835;
  
      // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = .00274;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kAutonPower = .5;
    public static final double kAutonLength = 1.5;

    public static final double kShooterLength = 5;
    }

}
 