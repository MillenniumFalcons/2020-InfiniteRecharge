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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import lib.drivers.TalonSRXUtil;
import lib.wpi.HALMethods;

/**
 * Add your docs here.
 */
public abstract class TalonSRXSubsystem implements Subsystem {

    protected PeriodicIO periodicIO;
    private TalonSRX master;
    private int m_masterCANID;
    private double[] m_PIDArr;
    private int m_maxVelocity, m_maxAcceleration;
    private double m_encoderThreshold;
    private int m_maxStallCurrent;
    private int m_maxCurrent;
    private int m_peakCurrentDuration;
    private boolean isInverted = false;
    private SubsystemControlType controlType = SubsystemControlType.OPENLOOP;
    private SimpleMotorFeedforward feedForwad;

    protected TalonSRXSubsystem(int masterCANID, double[] PIDArr, double[] feedForwardArr,
            int maxVelocity, int maxAcceleration, double encoderThreshold, int maxStallCurrent,
            int maxCurrent, int peakCurrentDuration, boolean inverted) {
        periodicIO = new PeriodicIO();
        m_masterCANID = masterCANID;
        m_PIDArr = PIDArr;
        m_maxVelocity = maxVelocity;
        m_maxAcceleration = maxAcceleration;
        m_maxStallCurrent = maxStallCurrent;
        m_maxCurrent = maxCurrent;
        m_peakCurrentDuration = peakCurrentDuration;
        isInverted = inverted;
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
            TalonSRXUtil.checkError(master.configContinuousCurrentLimit(m_maxStallCurrent),
                    "Couldn't set current limiting to " + getName() + " talonSRX");
            TalonSRXUtil.checkError(master.configPeakCurrentDuration(m_peakCurrentDuration),
                    "Couldn't set current limiting to " + getName() + " talonSRX");
            TalonSRXUtil.checkError(master.configPeakCurrentLimit(m_maxCurrent),
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

    public double getEncoderValue() {
        return periodicIO.encoderValue;
    }

    public double getEncoderVelocity() {
        return periodicIO.encoderVelocity;
    }

    public void resetEncoder() {
        periodicIO = new PeriodicIO();
    }

    protected void setEncoderValue(int newPosition) {
        TalonSRXUtil.checkError(master.setSelectedSensorPosition(newPosition),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    public void resetEncoderValue() {
        setEncoderValue(0);
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
        periodicIO.demand = refrencePt;
        periodicIO.feedforward = feedForwad.calculate(m_maxVelocity, m_maxAcceleration);
        controlType = SubsystemControlType.MOTIONMAGIC;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity;
        periodicIO.feedforward = feedForwad.calculate(velocity, m_maxAcceleration);
        controlType = SubsystemControlType.VELOCITY;
    }

    protected void setOpenloop(double demand) {
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
            int maxVelocity, int maxAcceleration, int slot) {
        TalonSRXUtil.checkError(master.config_kP(slot, PIDArr[0]), "Couldn't set P for " + name);
        TalonSRXUtil.checkError(master.config_kI(slot, PIDArr[1]), "Couldn't set I for " + name);
        TalonSRXUtil.checkError(master.config_kD(slot, PIDArr[2]), "Couldn't set D for " + name);
        TalonSRXUtil.checkError(master.config_kF(slot, PIDArr[3]), "Couldn't set F for " + name);

        TalonSRXUtil.checkError(master.configMotionAcceleration(maxAcceleration),
                "Couldn't set max accel for " + name);
        TalonSRXUtil.checkError(master.configMotionCruiseVelocity(maxVelocity),
                "Couldn't set max vel for " + name);
    }

    protected boolean reachedPosition(double position) {
        return position < getEncoderValue() + m_encoderThreshold
                && position > getEncoderValue() - m_encoderThreshold;
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

    protected void addFollower(TalonSRX talonFollower) {
        talonFollower.follow(master);
        talonFollower.setInverted(InvertType.FollowMaster);
    }

    protected void addFollower(VictorSPX victorFollower) {
        victorFollower.follow(master);
        victorFollower.setInverted(InvertType.FollowMaster);
    }

    public abstract String getName();
}
