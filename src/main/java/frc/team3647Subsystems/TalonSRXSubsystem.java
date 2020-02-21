/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import lib.drivers.TalonSRXUtil;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.ClosedLoopFactory;
import lib.drivers.TalonSRXFactory;
import lib.wpi.HALMethods;

/**
 * Add your own bound checking!
 */
public abstract class TalonSRXSubsystem implements PeriodicSubsystem {

    private TalonSRX master;
    private ControlMode controlMode = ControlMode.Disabled;
    private SimpleMotorFeedforward feedForwad;
    private TalonSRXFactory.Configuration m_masterConfig;
    private ClosedLoopConfig m_pidConfig;

    public static class PeriodicIO {
        // inputs
        public double position;
        public double velocity;
        public double current;

        // outputs
        public double feedforward;
        public double demand;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    protected TalonSRXSubsystem(TalonSRXFactory.Configuration masterConfig, ClosedLoopConfig pidConfig) {
        m_masterConfig = masterConfig;
        m_pidConfig = pidConfig;
        master = TalonSRXFactory.createTalon(m_masterConfig);
        ClosedLoopFactory.configTalonPIDController(master, FeedbackDevice.CTRE_MagEncoder_Relative, pidConfig, 0);
    }

    @Override
    public void init() {
        try {
            setToBrake();
            reconfigTalonPID();
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = TalonSRXFactory.createTalon(m_masterConfig);
            ClosedLoopFactory.configTalonPIDController(master, FeedbackDevice.CTRE_MagEncoder_Relative, m_pidConfig, 0);
        }
    }

    protected void reconfigTalonPID() {
        ClosedLoopFactory.configTalonPIDController(master, FeedbackDevice.CTRE_MagEncoder_Relative, m_pidConfig, 0);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = master.getSelectedSensorPosition() * m_pidConfig.kEncoderTicksToUnits;
        periodicIO.velocity = master.getSelectedSensorVelocity() * m_pidConfig.kEncoderVelocityToRPM;
        periodicIO.current = master.getStatorCurrent();
    }

    @Override
    public void writePeriodicOutputs() {
        master.set(controlMode, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedforward / 12.0);
    }

    @Override
    public void periodic() {
        writePeriodicOutputs();
        readPeriodicInputs();
    }

    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.feedforward = 0;
        periodicIO.demand = 0;
        controlMode = ControlMode.PercentOutput;
    }

    /**
     * @param newPosition set the encoder the this position, physical position will
     *                    not change
     */
    protected void setEncoderPosition(double newPosition) {
        TalonSRXUtil.checkError(master.setSelectedSensorPosition((int) newPosition),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    protected void setEncoderPositionUnits(double newPositionUnits) {
        setEncoderPosition(newPositionUnits / m_pidConfig.kEncoderTicksToUnits);
    }

    protected void resetEncoder() {
        setEncoderPosition(0);
    }

    /**
     * change the physical position of the subsystem based on units. Will physically
     * change where the subsystem is.
     * 
     * @param referencePt in real world units.
     */
    protected void setPosition(double referencePt) {
        periodicIO.demand = referencePt / m_pidConfig.kEncoderTicksToUnits;
        controlMode = ControlMode.Position;
    }

    /**
     * Will physically change where the subsystem is.
     * 
     * @param referencePt the reference pt in units as determined by scaler passed
     *                    in the constructor
     */
    protected void setPositionMotionMagic(double referencePt) {
        periodicIO.demand = referencePt / m_pidConfig.kEncoderTicksToUnits;
        controlMode = ControlMode.MotionMagic;
    }

    /**
     * @param velocity the velocity in RPM of the end effector
     */
    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity / m_pidConfig.kEncoderVelocityToRPM;
        double acceleration = (velocity - getVelocity()) / .02;
        periodicIO.feedforward = (feedForwad.calculate(velocity, acceleration) / 12.0)
                * (1023 / m_pidConfig.maxVelocity);
        controlMode = ControlMode.Velocity;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        periodicIO.feedforward = 0;
        controlMode = ControlMode.PercentOutput;
    }

    protected boolean reachedPosition(double position) {
        return position < getPosition() + m_pidConfig.positionThreshold
                && position > getPosition() - m_pidConfig.positionThreshold;
    }

    public boolean reachedTargetPosition() {
        if (controlMode == ControlMode.Position || controlMode == ControlMode.MotionMagic) {
            return reachedPosition(periodicIO.demand);
        }
        return false;
    }

    protected boolean reachedVelocity(double velocity) {
        return velocity < getVelocity() + m_pidConfig.velocityThreshold
                && velocity > getVelocity() - m_pidConfig.velocityThreshold;
    }

    public boolean reachedTargetVelocity() {
        if (controlMode == ControlMode.Velocity) {
            return reachedVelocity(getVelocity());
        }
        return false;
    }

    public double getPosition() {
        return periodicIO.position;
    }

    public double getVelocity() {
        return periodicIO.velocity;
    }

    public double getMasterCurrnet() {
        return periodicIO.current;
    }

    public void setToCoast() {
        try {
            master.setNeutralMode(NeutralMode.Coast);
        } catch (NullPointerException e) {
            master = TalonSRXFactory.createTalon(m_masterConfig);
            HALMethods.sendDSError(e.toString());
            master.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void setToBrake() {
        try {
            master.setNeutralMode(NeutralMode.Brake);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = TalonSRXFactory.createTalon(m_masterConfig);
            ClosedLoopFactory.configTalonPIDController(master, FeedbackDevice.CTRE_MagEncoder_Relative, m_pidConfig, 0);
        }
    }

    protected void addFollower(BaseMotorController follower, boolean isInvertedFromMaster) {
        follower.follow(master);
        InvertType invertType = InvertType.FollowMaster;
        if (isInvertedFromMaster) {
            invertType = InvertType.InvertMotorOutput;
        }
        follower.setInverted(invertType);
    }
}
