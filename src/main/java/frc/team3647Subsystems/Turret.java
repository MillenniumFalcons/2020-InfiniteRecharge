/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import lib.drivers.TalonSRXFactory;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;

/**
 * rotate.
 */
public class Turret extends TalonSRXSubsystem {
    private final double kMaxRotationDeg;
    private final double kMinRotationDeg;
    private DigitalInput limitSwitch;
    private boolean isOnLimitSwitch = false;

    public Turret(TalonSRXFactory.Configuration masterConfig, ClosedLoopConfig pidConfig, double maxRotationDeg,
            double minRotationDeg, int limitSwitchPin) {
        super(masterConfig, pidConfig);
        kMaxRotationDeg = maxRotationDeg;
        kMinRotationDeg = minRotationDeg;
        limitSwitch = new DigitalInput(limitSwitchPin);
        setEncoderPosition(0);
    }

    @Override
    public void periodic() {
        super.periodic();
        isOnLimitSwitch = !limitSwitch.get();
        if (isOnLimitSwitch) {
            // end();
        }
    }

    @Override
    protected void resetEncoder() {
        setEncoderPositionUnits(180);
    }

    /**
     * https://stackoverflow.com/a/2323034
     * 
     * @param angle any angle
     * @return if the angle is between the limits after normalized
     */
    private boolean isAngleGood(double angle) {
        angle = ((angle % 360) + 360) % 360;

        if (angle > 180) {
            angle -= 360;
        }

        return angle > kMinRotationDeg && angle < kMaxRotationDeg;
    }

    public void setAngle(double angle) {
        if (isAngleGood(angle)) {
            updatePositionFeedforward();
            setPosition(angle);
        } else if (isAngleTooBig(angle)) {
            setPosition(kMaxRotationDeg);
            updatePositionFeedforward();
        } else if (isAngleTooSmall(angle)) {
            setPosition(kMinRotationDeg);
            updatePositionFeedforward();
        } else {
            end();
        }

    }

    private boolean isAngleTooBig(double angle) {
        angle = ((angle % 360) + 360) % 360;

        if (angle > 180) {
            angle -= 360;
        }
        return angle >= kMaxRotationDeg;
    }

    private boolean isAngleTooSmall(double angle) {
        angle = ((angle % 360) + 360) % 360;

        if (angle > 180) {
            angle -= 360;
        }
        return angle <= kMinRotationDeg;
    }

    public void setAngleMotionMagic(double angle) {
        if (isAngleGood(angle)) {
            setPositionMotionMagic(angle);
        } else {
            end();
        }
    }

    public boolean isOnLimitSwitch() {
        return isOnLimitSwitch;
    }

    public double getAngle() {
        return getPosition();
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
