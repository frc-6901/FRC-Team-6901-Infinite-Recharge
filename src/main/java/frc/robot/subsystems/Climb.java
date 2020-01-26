package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climb extends SubsystemBase {

    private TalonSRX mClimbMaster;
    private VictorSPX mClimbSlave1, mClimbSlave2, mClimbSlave3;

    public Climb() {
        mClimbMaster = new TalonSRX(ClimberConstants.kClimberPort1);
        mClimbSlave1 = new VictorSPX(ClimberConstants.kClimberPort2);
        mClimbSlave2 = new VictorSPX(ClimberConstants.kClimberPort3);
        mClimbSlave3 = new VictorSPX(ClimberConstants.kClimberPort4);
        mClimbSlave1.follow(mClimbMaster);
        mClimbSlave2.follow(mClimbMaster);
        mClimbSlave3.follow(mClimbMaster);
    }

    public void runClimb(double power) {
        mClimbMaster.set(ControlMode.PercentOutput, power);

    }
}