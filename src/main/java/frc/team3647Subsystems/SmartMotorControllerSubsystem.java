/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import java.util.Objects;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * Add your docs here.
 */
public abstract class SmartMotorControllerSubsystem implements Subsystem {

    protected PeriodicIO periodicIO;
    protected int m_masterCANID;
    /**
     * Array that stores the PID constants for the subsystem: kP at [0], kI at [1] and kD at [2]
     */
    protected double[] m_PIDArr;
    /**
     * Array that stores the feed forward constants for the subsystem: kS at [0], kV at [1] and kA
     * at [2]
     */
    protected double[] m_FFArr;

    /**
     * the max velocity in native units of the subsystem
     */
    protected double m_maxVelocity;
    /**
     * the max acceleration in native units of the subsystem
     */
    protected double m_maxAcceleration;

    /**
     * the allowable encoder threshold from a set reference point in native position units
     */
    protected double m_EncoderPositionThreshold;
    /**
     * the allowable encoder threshold from a set reference point in native velocity units
     */
    protected double m_EncoderVelocityThreshold;

    protected boolean isInverted = false;

    /**
     * the conversion factor of native velocity to RPM of end product
     */
    protected double kEncoderVelocityToRPM;
    /**
     * the conversion factor of native position to distance or degress
     */
    protected double kEncoderPositionToUnits;

    protected SimpleMotorFeedforward feedforward;

    /**
     * @param masterCANID             the CAN ID of the master motor controller (1-62)
     * @param kEncoderVelocityToRPM   the conversion factor of native velocity to RPM of end product
     * @param kEncoderPositionToUnits the conversion factor of native position to distance or
     *                                degress
     * @param PIDArr                  Array that stores the PID constants for the subsystem: kP at
     *                                [0], kI at [1] and kD at [2]
     * @param feedForwardArr          Array that stores the feed forward constants for the
     *                                subsystem: kS at [0], kV at [1] and kA at [2]
     * @param maxVelocity             the max velocity in RPM of end product
     * @param maxAcceleration         the max acceleration in REV/s^2
     * @param positionThreshold       the max allowable error of the subsystem away from its
     *                                setpoint
     * @param velocityThreshold       the allowable error when setting closed loop velocity, in RPM
     * @param inverted                is the subsystem motor inverted
     */
    protected SmartMotorControllerSubsystem(int masterCANID, double kEncoderAccelerationToUnits,
            double kEncoderVelocityToRPM, double kEncoderPositionToUnits, double[] PIDArr,
            double[] feedForwardArr, double maxVelocity, double maxAcceleration,
            double positionThreshold, double velocityThreshold, boolean inverted) {
        periodicIO = new PeriodicIO();
        assert masterCANID != 0;
        m_masterCANID = masterCANID;

        m_PIDArr = Objects.requireNonNull(PIDArr);
        assert m_PIDArr.length > 2;

        m_FFArr = Objects.requireNonNull(feedForwardArr);
        assert m_FFArr.length > 2;

        assert kEncoderVelocityToRPM != 0;
        m_maxVelocity = maxVelocity / kEncoderVelocityToRPM;
        assert m_maxVelocity != 0;

        assert kEncoderAccelerationToUnits != 0;
        m_maxAcceleration = maxAcceleration / kEncoderAccelerationToUnits; // need to convert units
        assert m_maxAcceleration != 0;

        isInverted = inverted;

        assert kEncoderPositionToUnits != 0;
        m_EncoderPositionThreshold = positionThreshold / kEncoderPositionToUnits;
        assert m_EncoderPositionThreshold != 0;

        m_EncoderVelocityThreshold = velocityThreshold / kEncoderVelocityToRPM;
        assert m_EncoderVelocityThreshold != 0;

        this.kEncoderPositionToUnits = kEncoderPositionToUnits;
        this.kEncoderVelocityToRPM = kEncoderVelocityToRPM;

        feedforward = new SimpleMotorFeedforward(m_FFArr[0], m_FFArr[1], m_FFArr[2]);
    }

    /**
     * @return position in real world units, depends on the conversion factor passed to the
     *         consturctor
     */
    public double getPosition() {
        return periodicIO.encoderValue * kEncoderPositionToUnits;
    }

    /**
     * @return velocity in RPM of the end product after gear reduction
     */
    public double getVelocity() {
        return periodicIO.encoderVelocity * kEncoderVelocityToRPM;
    }


    /**
     * PARAMETERS ARE FOR NEO
     * 
     * @param maxFreeSpeedCurrent
     * @param maxStallCurrent
     */
    protected void setCurrentLimiting(int maxStallCurrent, int maxFreeSpeedCurrent) {
        System.out.println("method does nothing, must override!");
    }

    /**
     * PARAMETERS ARE FOR TALON!
     * 
     * @param peakCurrent
     * @param peakCurrentDuration
     * @param continuousCurrent
     */
    protected void setCurrentLimiting(int peakCurrent, int peakCurrentDuration,
            int continuousCurrent) {
        System.out.println("method does nothing, must override!");
    }

    /**
     * make encoder value 0
     */
    public void resetEncoder() {
        setEncoderPosition(0);
        periodicIO = new PeriodicIO();
    }

    /**
     * @param position the position in real world units to check against
     * @return has the encoder reached the threshold set in the constructor
     */
    protected boolean reachedPosition(double position) {
        // convert position to native units
        position /= kEncoderPositionToUnits;

        return position < periodicIO.encoderValue + m_EncoderPositionThreshold
                && position > periodicIO.encoderValue - m_EncoderPositionThreshold;
    }

    /**
     * @param encoderPosition the position in real world units of the subsystem
     */
    protected abstract void setEncoderPosition(double encoderPosition);

    /**
     * @param position the position in real world units, depends on the conversion factor passed to
     *                 the constuctor
     */
    protected abstract void setPosition(double position);

    /**
     * @param velocity the velocity in RPM of the end product after gear reduction, depends on the
     *                 conversion factor passed to the constuctor
     */
    protected abstract void setVelocity(double velocity);

    /**
     * @param demand -1 to 1 percent output
     */
    public abstract void setOpenloop(double demand);

    public abstract void setToCoast();

    public abstract void setToBrake();


    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.feedforward = 0;
    }

    /**
     * this does not check for parameter validity, you must do it yourself
     */
    protected abstract <T> void addFollower(T follower);

    public class PeriodicIO {
        public double encoderValue;
        public double encoderVelocity;
        public boolean hasResetOccured;
        public double timestamp;

        public double demand;
        public double feedforward;
    }

    public abstract String getName();
}
