package frc.team3647Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import lib.drivers.SparkMaxUtil;
import lib.wpi.HALMethods;

public abstract class SparkMaxSubsystem implements Subsystem {

    protected PeriodicIO periodicIO;
    private CANSparkMax master;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private int m_masterCANID;
    private double[] m_PIDArr;
    private double m_maxVelocity, m_maxAcceleration;
    private double m_encoderThreshold;
    private int m_maxStallCurrent;
    private int m_maxCurrent;
    private boolean isInverted = false;
    private ControlType controlType = ControlType.kDutyCycle;

    protected SparkMaxSubsystem(int masterCANID, double[] PIDArr, double feedForwardArr , double maxVelocity,
            double maxAcceleration, double encoderThreshold, int maxStallCurrent, int maxCurrent,
            boolean inverted) {
        periodicIO = new PeriodicIO();
        m_masterCANID = masterCANID;
        m_PIDArr = PIDArr;
        m_maxVelocity = maxVelocity;
        m_maxAcceleration = maxAcceleration;
        m_maxStallCurrent = maxStallCurrent;
        m_maxCurrent = maxCurrent;
        isInverted = inverted;
        master = new CANSparkMax(m_masterCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANEncoder(master);
        pidController = new CANPIDController(master);

    }

    @Override
    public void init() {
        try {
            master.setInverted(isInverted);
            setToBrake();
            SparkMaxUtil.checkError(pidController.setFeedbackDevice(encoder),
                    "Couldn't set " + getName() + " sparkmax hall sensor to pid feedback");
            configPID(pidController, getName() + " master sparkmax ", m_PIDArr, m_maxVelocity,
                    m_maxAcceleration, 0);
            SparkMaxUtil.checkError(master.setSmartCurrentLimit(m_maxStallCurrent, m_maxCurrent),
                    "Couldn't set current limiting to " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = new CANSparkMax(m_masterCANID, MotorType.kBrushless);
            master.setInverted(isInverted);
            setToBrake();
            pidController = new CANPIDController(master);
            SparkMaxUtil.checkError(pidController.setFeedbackDevice(encoder),
                    "Couldn't set sparkmax hall sensor to pid feedback");
            configPID(pidController, getName() + " master sparkmax ", m_PIDArr, m_maxVelocity,
                    m_maxAcceleration, 0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.encoderValue = encoder.getPosition();
        periodicIO.encoderVelocity = encoder.getVelocity();
    }

    @Override
    public void writePeriodicOutputs() {
        setSparkMax(controlType, periodicIO.demand, periodicIO.feedforward);
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

    protected void setEncoderValue(double newPosition) {
        SparkMaxUtil.checkError(encoder.setPosition(newPosition),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    public void resetEncoderValue() {
        setEncoderValue(0);
    }

    protected enum SubsystemControlType {
        OPENLOOP, SMARTMOTION, VELOCITY
    }

    protected void setPosition(double refrencePt) {
        periodicIO.demand = refrencePt;
        controlType = ControlType.kSmartMotion;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity;
        controlType = ControlType.kSmartMotion;
    }

    protected void setOpenloop(double demand) {
        periodicIO.demand = demand;
        controlType = ControlType.kDutyCycle;
    }

    public class PeriodicIO {
        public double encoderValue;
        public double encoderVelocity;
        public boolean hasResetOccured;
        public double timestamp;

        public double demand;
        public double feedforward;
    }

    private synchronized void configPID(CANPIDController controller, String name, double[] PIDArr,
            double maxVelocity, double maxAcceleration, int slot) {
        SparkMaxUtil.checkError(controller.setP(PIDArr[0], slot), "Couldn't set P for " + name);
        SparkMaxUtil.checkError(controller.setI(PIDArr[1], slot), "Couldn't set I for " + name);
        SparkMaxUtil.checkError(controller.setD(PIDArr[2], slot), "Couldn't set D for " + name);

        SparkMaxUtil.checkError(controller.setSmartMotionMaxAccel(maxAcceleration, slot),
                "Couldn't set max accel for " + name);
        SparkMaxUtil.checkError(controller.setSmartMotionMaxVelocity(maxVelocity, slot),
                "Couldn't set max vel for " + name);
    }

    protected boolean reachedPosition(double position) {
        return position < getEncoderValue() + m_encoderThreshold && position > getEncoderValue() - m_encoderThreshold;
    }
    
    public void setToCoast() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kCoast),
                    " Couldn't set to coast " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = new CANSparkMax(m_masterCANID, MotorType.kBrushless);
            HALMethods.sendDSError(e.toString());
        }
    }

    public void setToBrake() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kBrake),
                    " Couldn't set to brake " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = new CANSparkMax(m_masterCANID, MotorType.kBrushless);
            HALMethods.sendDSError(e.toString());
        }
    }

    private void setSparkMax(ControlType controlType, double demand, double feedForward) {
        pidController.setReference(demand, controlType, 0, feedForward);
    }

    protected void addFollower(CANSparkMax follower) {
        follower.follow(master);
        follower.setInverted(master.getInverted());
    }


    public abstract String getName();

}
