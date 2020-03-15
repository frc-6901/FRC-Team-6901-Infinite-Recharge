package frc.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ControllerWrapper;

public class XboxJoystickTrigger extends Trigger {
    private ControllerWrapper mController;
    private Hand kHand;
    private Position mPosition;


    public XboxJoystickTrigger(ControllerWrapper controller, Hand hand, Position position) {
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
                return mController.getY(kHand) > 0;
            case DOWN:
                return mController.getY(kHand) < 0;
            case LEFT:
                return mController.getX(kHand) < 0;
            case RIGHT:
                return mController.getX(kHand) > 0;
            default:
                return false; 
        }
    }
}