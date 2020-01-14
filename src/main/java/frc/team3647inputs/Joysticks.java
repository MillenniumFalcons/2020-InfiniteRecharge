/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647inputs;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Add your docs here.
 */
public class Joysticks {
    /**
     * Main controller Variable
     */
    public double leftTrigger, rightTrigger, leftJoyStickY, leftJoyStickX, rightJoyStickY,
            rightJoyStickX, rumbleIntensity;
    public boolean rightBumper, leftBumper, buttonA, buttonB, buttonY, buttonX, dPadDown, dPadLeft,
            dPadRight, dPadUp;
    public boolean rightJoyStickPress, leftJoyStickPress, leftMidButton, rightMidButton,
            buttonXPressed;

    /**
     * XboxController Object for Main-Driver Controller; contains all Xbox Controller Functions
     */
    private XboxController controller;

    private int controllerPin;

    /**
     * Co-Driver Controller Variable
     */
    private int dPadValue = -1; // dPad degree value

    public Joysticks(int controllerPin) {
        controller = new XboxController(controllerPin);
        this.controllerPin = controllerPin;
    }

    /**
     * update main controller values, must be run to use public variables as triggers
     */
    public void update() {
        leftBumper = controller.getBumper(XboxController.Hand.kLeft);
        rightBumper = controller.getBumper(XboxController.Hand.kRight);
        leftTrigger = joystickThreshold(controller.getTriggerAxis(XboxController.Hand.kLeft));
        rightTrigger = joystickThreshold(controller.getTriggerAxis(XboxController.Hand.kRight));
        buttonA = controller.getAButton();
        buttonB = controller.getBButton();
        buttonX = controller.getXButton();
        buttonXPressed = controller.getXButtonPressed();
        buttonY = controller.getYButton();

        leftJoyStickX = joystickThreshold(controller.getX(XboxController.Hand.kLeft));
        leftJoyStickY = joystickThreshold(-controller.getY(XboxController.Hand.kLeft));
        rightJoyStickX = joystickThreshold(controller.getX(XboxController.Hand.kRight));
        rightJoyStickY = joystickThreshold(-controller.getY(XboxController.Hand.kRight));

        rightJoyStickPress = controller.getStickButton(XboxController.Hand.kRight);
        leftJoyStickPress = controller.getStickButton(XboxController.Hand.kLeft);

        leftMidButton = controller.getBackButton();
        rightMidButton = controller.getStartButton();

        controller.setRumble(RumbleType.kLeftRumble, rumbleIntensity);
        controller.setRumble(RumbleType.kRightRumble, rumbleIntensity);

        setDPadValues(controller.getPOV());
    }

    /**
     * Set co driver dPad values. 0 degrees = top, 180 = down, 90 right 270 == left
     */
    private void setDPadValues(int povValue) {
        if (dPadValue == 0) {
            dPadUp = true;
            dPadDown = false;
            dPadLeft = false;
            dPadRight = false;
        }
        if (dPadValue == 180) {
            dPadUp = false;
            dPadDown = true;
            dPadLeft = false;
            dPadRight = false;
        }
        if (dPadValue == 90) {
            dPadUp = false;
            dPadDown = false;
            dPadLeft = false;
            dPadRight = true;
        }
        if (dPadValue == 270) {
            dPadUp = false;
            dPadDown = false;
            dPadLeft = true;
            dPadRight = false;
        }
        if (dPadValue == -1) {
            dPadUp = false;
            dPadDown = false;
            dPadLeft = false;
            dPadRight = false;
        }
    }

    /**
     * 
     * @param jValue is the joystick value input
     * @return returns joystick value if outside of joystick threshold, else returns zero
     */
    public static double joystickThreshold(double value) {
        return Math.abs(value) < .09 ? 0 : value;
    }

    public int getControllerPin() {
        return controllerPin;
    }

}

