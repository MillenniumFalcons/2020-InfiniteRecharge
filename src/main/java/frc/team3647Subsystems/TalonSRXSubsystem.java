/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import lib.drivers.TalonSRXUtil;
import lib.wpi.HALMethods;

/**
 * Add your docs here.
 */
public abstract class TalonSRXSubsystem extends SmartMotorControllerSubsystem {

    private TalonSRX master;
    private SubsystemControlType controlType = SubsystemControlType.OPENLOOP;
    private SimpleMotorFeedforward feedForwad;
    private int m_continuousCurrent;
    private int m_peakCurrentDuration;
    private int m_peakCurrent;

    protected TalonSRXSubsystem(int masterCANID, double kEncoderAccelerationToUnits,
            double kEncoderVelocityToRPM, double kEncoderPositionToUnits, double[] PIDArr,
            double[] feedForwardArr, double maxVelocity, double maxAcceleration,
            double positionThreshold, double velocityThreshold, boolean inverted, int peakCurrent,
            int continuousCurrent, int peakCurrentDuration) {
        super(masterCANID, kEncoderAccelerationToUnits, kEncoderVelocityToRPM,
                kEncoderPositionToUnits, PIDArr, feedForwardArr, maxVelocity, maxAcceleration,
                positionThreshold, velocityThreshold, inverted);

        assert continuousCurrent != 0;
        m_continuousCurrent = continuousCurrent;

        assert peakCurrent != 0;
        m_peakCurrent = peakCurrent;

        master = new TalonSRX(m_masterCANID);
        feedForwad =
                new SimpleMotorFeedforward(feedForwardArr[0], feedForwardArr[1], feedForwardArr[2]);

    }

    @Override
    public void init() {
        try {
            master.setInverted(isInverted);
            setToBrake();
            master.enableCurrentLimit(true);
            TalonSRXUtil.checkError(master.configContinuousCurrentLimit(m_continuousCurrent),
                    "Couldn't set current limiting to " + getName() + " talonSRX");
            TalonSRXUtil.checkError(master.configPeakCurrentDuration(m_peakCurrentDuration),
                    "Couldn't set current limiting to " + getName() + " talonSRX");
            TalonSRXUtil.checkError(master.configPeakCurrentLimit(m_peakCurrent),
                    "Couldn't set current limiting to " + getName() + " talonSRX");
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = new TalonSRX(m_masterCANID);
            master.setInverted(isInverted);
            setToBrake();

            configPID(master, getName() + " master talonSRX ", m_PIDArr, m_maxVelocity,
                    m_maxAcceleration, 0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.encoderValue = master.getSelectedSensorPosition();
        periodicIO.encoderVelocity = master.getSelectedSensorVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        master.set(controlType.controlMode, periodicIO.demand, DemandType.ArbitraryFeedForward,
                periodicIO.feedforward);
    }

    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.feedforward = 0;
    }

    @Override
    public void periodic(double timestamp) {
        periodicIO.timestamp = timestamp;
    }

    protected void setEncoderPosition(double newPosition) {
        TalonSRXUtil.checkError(master.setSelectedSensorPosition((int)newPosition),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }


    protected enum SubsystemControlType {
        OPENLOOP(ControlMode.PercentOutput), MOTIONMAGIC(ControlMode.MotionMagic), POSITION(
                ControlMode.Position), VELOCITY(ControlMode.Velocity);

        public ControlMode controlMode;

        SubsystemControlType(ControlMode controlMode) {
            this.controlMode = controlMode;
        }
    }

    protected void setPosition(double refrencePt) {
        periodicIO.demand = refrencePt * kEncoderPositionToUnits;
        periodicIO.feedforward = feedForwad.calculate(m_maxVelocity, m_maxAcceleration);
        controlType = SubsystemControlType.MOTIONMAGIC;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity / kEncoderVelocityToRPM;
        periodicIO.feedforward = feedForwad.calculate(velocity, m_maxAcceleration);
        controlType = SubsystemControlType.VELOCITY;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        periodicIO.feedforward = 0;
        controlType = SubsystemControlType.OPENLOOP;
    }

    public class PeriodicIO {
        public int encoderValue;
        public int encoderVelocity;
        public boolean hasResetOccured;
        public double timestamp;

        public double demand;
        public double feedforward;
    }

    private synchronized void configPID(TalonSRX master, String name, double[] PIDArr,
            double maxVelocity, double maxAcceleration, int slot) {
        TalonSRXUtil.checkError(master.config_kP(slot, PIDArr[0]), "Couldn't set P for " + name);
        TalonSRXUtil.checkError(master.config_kI(slot, PIDArr[1]), "Couldn't set I for " + name);
        TalonSRXUtil.checkError(master.config_kD(slot, PIDArr[2]), "Couldn't set D for " + name);
        TalonSRXUtil.checkError(master.config_kF(slot, PIDArr[3]), "Couldn't set F for " + name);

        TalonSRXUtil.checkError(master.configMotionAcceleration((int) maxAcceleration),
                "Couldn't set max accel for " + name);
        TalonSRXUtil.checkError(master.configMotionCruiseVelocity((int) maxVelocity),
                "Couldn't set max vel for " + name);
    }

    protected boolean reachedPosition(double position) {
        return position < getPosition() + m_EncoderPositionThreshold * kEncoderPositionToUnits
                && position > getPosition() - m_EncoderPositionThreshold * kEncoderPositionToUnits;
    }

    public void setToCoast() {
        try {
            master.setNeutralMode(NeutralMode.Coast);
        } catch (NullPointerException e) {
            master = new TalonSRX(m_masterCANID);
            HALMethods.sendDSError(e.toString());
            master.setNeutralMode(NeutralMode.Coast);
        }
    }

    public void setToBrake() {
        try {
            master.setNeutralMode(NeutralMode.Brake);
        } catch (NullPointerException e) {
            master = new TalonSRX(m_masterCANID);
            HALMethods.sendDSError(e.toString());
            master.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    protected <T> void addFollower(T follower) {
        if (follower instanceof BaseMotorController) {
            BaseMotorController cFollower = (BaseMotorController) follower;
            cFollower.follow(master);
            cFollower.setInverted(InvertType.FollowMaster);
        }
    }
}
