package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public class XboxJoystickTrigger extends Trigger {
    XboxController mController;
    Hand kHand;
    Position mPosition;
    public XboxJoystickTrigger(XboxController controller, Hand hand, Position position) {
        mController = controller;
        kHand = hand;
        mPosition = position;
    }

    public enum Position {
        UP,
        DOWN,
        LEFT,
        RIGHT
    }
    @Override
    public boolean get() {
        
        switch(mPosition) {
            case UP:
                return mController.getY(kHand) > ControllerConstants.deadBand;
            case DOWN:
                return mController.getY(kHand) < -ControllerConstants.deadBand;
            case LEFT:
                return mController.getX(kHand) < -ControllerConstants.deadBand;
            case RIGHT:
                return mController.getX(kHand) > ControllerConstants.deadBand;
            default:
                return false; 
        }
    }
}