package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FeederConstants;

public class Feeder extends SubsystemBase {

    
    private VictorSPX mIndexerMotor, mAcceleratorWheel;

    public Feeder() {
        mIndexerMotor = new VictorSPX(FeederConstants.kIndexerPort);
        mAcceleratorWheel = new VictorSPX(FeederConstants.kAcceleratorWheelPort);
        //mIndexerMotor2 = new VictorSPX(ClimberConstants.kClimberPort3);
        
    }

    public void runIndexer() {
        mIndexerMotor.set(ControlMode.PercentOutput, FeederConstants.kIndexerSpeed);
        //mIndexerMotor2.set(ControlMode.PercentOutput, power);
    }

    public void runAccelerator() {
        mAcceleratorWheel.set(ControlMode.PercentOutput, FeederConstants.kAcceleratorSpeed);
    }

    public void runFeeder() {
        runIndexer();
        runAccelerator();
    }

    public void reverseFeeder() {
        mAcceleratorWheel.set(ControlMode.PercentOutput, -FeederConstants.kAcceleratorSpeed);
        mIndexerMotor.set(ControlMode.PercentOutput, -FeederConstants.kIndexerSpeed);
    }

    public void stopFeeder() {
        mIndexerMotor.set(ControlMode.PercentOutput, 0);
        mAcceleratorWheel.set(ControlMode.PercentOutput, 0);
    }
}