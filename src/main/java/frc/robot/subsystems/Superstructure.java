/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;

public class Superstructure extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  private Shooter mShooter;
  private Feeder mFeeder;
  private SuperstructureState mState = SuperstructureState.IDLE;
  private VisionController mLimelight;
  private int mShootingState = 0;

  public enum SuperstructureState {
      IDLE,
      DEFAULT_SHOOTING,
      TUNING_SHOOTER,
      LONG_SHOT,
      LIMELIGHT_SHOOTING
  }

  public Superstructure(Shooter shooter, Feeder feeder, VisionController limelight) {
    mShooter = shooter;
    mFeeder = feeder;
    mLimelight = limelight;
  }

  public void shoot(SuperstructureState state) {
      mState = state;
      mShootingState = 0; 
  }

  public void stop() {
      mState = SuperstructureState.IDLE;
  }

  public Shooter getShooter() {
      return mShooter;
  }

  public VisionController getLimelight() {
      return mLimelight;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (mState) {
        case DEFAULT_SHOOTING:
            switch(mShootingState) {
                case 0:
                    mShooter.variableRPMShooter(ShooterConstants.kDefaultRPM);
                    if (mShooter.atSpeed()) {
                        mShootingState++;
                    } else {
                        break;
                    }
                case 1:
                    mFeeder.runFeeder();
                    break;
            
            }
            break;
        case LONG_SHOT:
            switch(mShootingState) {
                case 0:
                    mShooter.longShot();
                    if (mShooter.atSpeed()) {
                        mShootingState++;
                    } else {
                        break;
                    }
                case 1:
                    mFeeder.runFeeder();
                    break;
        
            }
            break;
        case TUNING_SHOOTER:
            switch(mShootingState) {
                case 0:
                    mShooter.tuningRPMShooter(ShooterConstants.kDefaultRPM);
                    if (mShooter.atSpeed()) {
                        mShootingState++;
                    } else {
                        break;
                    }
                case 1:
                    mFeeder.runFeeder();
                    break;
        
        }
        
            break;
        case LIMELIGHT_SHOOTING:
            switch (mShootingState) {
                case 0:
                    //int targetRPM = mLimelight.getRPM();
                    //SmartDashboard.putNumber("Target RPMM", targetRPM);
                    mShooter.variableRPMShooter(mLimelight.getRPM());
                    if (mShooter.atSpeed()) {
                        mShootingState++;
                    } else {
                        break;
                    }
                case 1:
                    mFeeder.runFeeder();
                    break;

            }
            break;
        default:
            mShooter.stopShooter();
            mFeeder.stopFeeder();
            break;


    }

  }
}
