package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IndexerConstants;

public class Feeder extends SubsystemBase {

    
    private VictorSPX mIndexerMotor1;

    public Feeder() {
        mIndexerMotor1 = new VictorSPX(IndexerConstants.kIndexerPort1);
        //mIndexerMotor2 = new VictorSPX(ClimberConstants.kClimberPort3);
        
    }

    public void runIndexer(double power) {
        mIndexerMotor1.set(ControlMode.PercentOutput, power);
        //mIndexerMotor2.set(ControlMode.PercentOutput, power);
    }

    public void runFeeder() {
        runIndexer(0);
    }

    public void stopFeeder() {
        mIndexerMotor1.set(ControlMode.PercentOutput, 0);
    }
}