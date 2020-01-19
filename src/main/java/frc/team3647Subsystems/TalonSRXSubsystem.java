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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.drivers.TalonSRXUtil;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.ClosedLoopFactory;
import lib.drivers.TalonSRXFactory;
import lib.wpi.HALMethods;

/**
 * Add your docs here.
 */
public abstract class TalonSRXSubsystem extends SubsystemBase implements PeriodicSubsystem {

    private TalonSRX master;
    private ControlMode controlMode = ControlMode.Disabled;
    private SimpleMotorFeedforward feedForwad;
    private TalonSRXFactory.Configuration m_masterConfig;
    private ClosedLoopConfig m_pidConfig;

    public static class PeriodicIO {
        // inputs
        public double position;
        public double velocity;

        // outputs
        public double feedforward;
        public double demand;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    protected TalonSRXSubsystem(TalonSRXFactory.Configuration masterConfig,
            ClosedLoopConfig pidConfig) {
        m_masterConfig = masterConfig;
        m_pidConfig = pidConfig;
        master = TalonSRXFactory.createTalon(m_masterConfig);
        ClosedLoopFactory.configTalonPIDController(master, FeedbackDevice.CTRE_MagEncoder_Relative,
                pidConfig, 0);
    }

    @Override
    public void init() {
        try {
            setToBrake();
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = TalonSRXFactory.createTalon(m_masterConfig);
            ClosedLoopFactory.configTalonPIDController(master,
                    FeedbackDevice.CTRE_MagEncoder_Relative, m_pidConfig, 0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = master.getSelectedSensorPosition();
        periodicIO.velocity = master.getSelectedSensorVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        master.set(controlMode, periodicIO.demand, DemandType.ArbitraryFeedForward,
                periodicIO.feedforward / 12.0);
    }

    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.feedforward = 0;
    }

    @Override
    public void periodic() {

    }

    /**
     * @param newPosition set the encoder the this position, physical position will not change
     */
    protected void setEncoderPosition(double newPosition) {
        TalonSRXUtil.checkError(master.setSelectedSensorPosition((int) newPosition),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    /**
     * change the physical position of the subsystem based on units. Will physically change where
     * the subsystem is.
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
     * @param referencePt the reference pt in units as determined by scaler passed in the
     *                    constructor
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

    public double getPosition() {
        return periodicIO.position;
    }

    public double getVelocity() {
        return periodicIO.velocity;
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
            ClosedLoopFactory.configTalonPIDController(master,
                    FeedbackDevice.CTRE_MagEncoder_Relative, m_pidConfig, 0);
        }
    }

    protected <T> void addFollower(T follower) {
        if (follower instanceof BaseMotorController) {
            BaseMotorController cFollower = (BaseMotorController) follower;
            cFollower.follow(master);
            cFollower.setInverted(InvertType.FollowMaster);
        }
    }
}
