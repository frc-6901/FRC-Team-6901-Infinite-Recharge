package frc.Util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxPOVButton extends Trigger {
    private XboxController mXboxController;
    private Arrow mButton;

    public XboxPOVButton(XboxController xboxController, Arrow button) {
        mButton = button;
        mXboxController = xboxController;
    }

    public enum Arrow {
        UP,
        RIGHT,
        LEFT,
        DOWN,
        CENTER
    }

    @Override
    public boolean get() {
        int targetValue = 0;
        switch(mButton) {
            case DOWN:
                targetValue = 180;
                break;
            case LEFT:
                targetValue = 270;
                break;
            case RIGHT:
                targetValue = 90;
                break;
            case CENTER:
                targetValue = -1;
                break;
            case UP:
            default:
                break;
        }
        return mXboxController.getPOV() == targetValue;
    }

}