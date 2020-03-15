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
    }

    public void runIndexer() {
        mIndexerMotor.set(ControlMode.Current, FeederConstants.kIndexerVolts);
    }

    public void runAccelerator() {
        mAcceleratorWheel.set(ControlMode.Current, -FeederConstants.kAcceleratorVolts);
    }

    public void runFeeder() {
        runIndexer();
        runAccelerator();
    }

    public void reverseFeeder() {
        mAcceleratorWheel.set(ControlMode.Current, FeederConstants.kAcceleratorVolts);
        mIndexerMotor.set(ControlMode.Current, -FeederConstants.kIndexerVolts);
    }

    public void stopFeeder() {
        mIndexerMotor.set(ControlMode.Current, 0);
        mAcceleratorWheel.set(ControlMode.Current, 0);
    }
}