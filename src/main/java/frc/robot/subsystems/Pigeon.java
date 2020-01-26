package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

public class Pigeon {
    PigeonIMU pigeon;
    public Pigeon(int portNumber) {
        pigeon = new PigeonIMU(portNumber);
        
    }

    public double getAngle() {
        PigeonState state = pigeon.getState();
        if (state == PigeonState.Ready) {
            double[] ypr = new double[3];
            pigeon.getYawPitchRoll(ypr);
            return ypr[0];
        } else {
            System.out.println("Pigeon is not ready!");
            return 0;

        }
          
    }
    public void reset() {
        System.out.println("Reset Gyro!");
        pigeon.setYaw(0);
    }



}