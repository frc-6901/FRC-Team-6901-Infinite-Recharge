package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShooterConstants;

public class VisionController extends SubsystemBase{

    private double[] limelightValues = new double[2];
    private boolean seesTarget = false;
    private double prevError = 0;

    private static VisionController mInstance = null;

    public static VisionController getInstance() {
        if (mInstance == null) {
            mInstance = new VisionController();
        }

        return mInstance;
    }
    private VisionController() {
        SmartDashboard.putNumber("kP", DriveConstants.kP);
        SmartDashboard.putNumber("kD", DriveConstants.kD);
        
    }

    private void updateValues() {
        seesTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1;
        limelightValues[0] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        limelightValues[1] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    public void lightsOn() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    public void lightsOff() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }
    
    private double estimateDistance() {
        //updateValues();
        double heightDistance = LimelightConstants.kTargetHeight - LimelightConstants.kLimelightHeight;
        double totalAngle = LimelightConstants.kLimelightAngle + limelightValues[1];

        return heightDistance / Math.tan(Math.toRadians(totalAngle));  
    }

    public double getDriveOutput(double kP, double kD) {
        double error = limelightValues[0];
        double output = kP * error + kD * (error - prevError);
        prevError = error;
        return output;
    }

    public double tuneDriveLimelight() {
        double kP = SmartDashboard.getNumber("kP", DriveConstants.kP);
        double kD = SmartDashboard.getNumber("kD", DriveConstants.kD);
        return getDriveOutput(kP, kD);
    }

    public double getXOffset() {
        return limelightValues[0];
    }

    public boolean isAligned() {
        return Math.abs(limelightValues[0]) <= LimelightConstants.kXThreshold && prevError == limelightValues[0]; // checks to see if it is aligned and not moving 
    }

    public int getRPM() {

        double distance = estimateDistance();
        return (int) ShooterConstants.kPolynomial.predict(distance);
    }

    @Override
    public void periodic() {
        updateValues();
        SmartDashboard.putBoolean("Sees Target", seesTarget);
        SmartDashboard.putNumber("Target Y Error", limelightValues[1]);
        SmartDashboard.putNumber("Estimate Distance", estimateDistance());
        SmartDashboard.putNumber("Target RPM", getRPM());
    }

}