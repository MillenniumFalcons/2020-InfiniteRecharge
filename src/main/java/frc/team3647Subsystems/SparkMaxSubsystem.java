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

public abstract class SparkMaxSubsystem extends SmartMotorControllerSubsystem {

    private CANSparkMax master;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private ControlType controlType = ControlType.kDutyCycle;
    private int m_maxFreeSpeedCurrent;
    private int m_maxStallCurrent;

    protected SparkMaxSubsystem(int masterCANID, double kEncoderAccelerationToUnits,
            double kEncoderVelocityToRPM, double kEncoderPositionToUnits, double[] PIDArr,
            double[] feedForwardArr, double maxVelocity, double maxAcceleration,
            double positionThreshold, double velocityThreshold, boolean inverted,
            int maxFreeSpeedCurrent, int maxStallCurrent) {
        super(masterCANID, kEncoderAccelerationToUnits, kEncoderVelocityToRPM,
                kEncoderPositionToUnits, PIDArr, feedForwardArr, maxVelocity, maxAcceleration,
                positionThreshold, velocityThreshold, inverted);

        master = new CANSparkMax(m_masterCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
        encoder = new CANEncoder(master);
        pidController = new CANPIDController(master);

        assert m_maxFreeSpeedCurrent != 0;
        m_maxFreeSpeedCurrent = maxFreeSpeedCurrent;

        assert m_maxStallCurrent != 0;
        m_maxStallCurrent = maxStallCurrent;
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
            SparkMaxUtil.checkError(master.setSmartCurrentLimit(m_maxStallCurrent, m_maxFreeSpeedCurrent),
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

    protected void setEncoderPosition(double newPosition) {
        SparkMaxUtil.checkError(encoder.setPosition(newPosition / kEncoderPositionToUnits),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    protected void setPosition(double refrencePt) {
        periodicIO.demand = refrencePt / kEncoderPositionToUnits;
        controlType = ControlType.kSmartMotion;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity / kEncoderVelocityToRPM;
        controlType = ControlType.kSmartMotion;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        controlType = ControlType.kDutyCycle;
    }

    @Override
    protected void setCurrentLimiting(int maxStallCurrent, int maxFreeSpeedCurrent) {
        SparkMaxUtil.checkError(master.setSmartCurrentLimit(maxStallCurrent, maxFreeSpeedCurrent),
                "Couldn't set current limiting to " + getName() + " sparkmax");
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

    /**
     * @param position position to check in real world units
     */
    protected boolean reachedPosition(double position) {
        return position < getPosition() + m_EncoderPositionThreshold * kEncoderPositionToUnits
                && position > getPosition() - m_EncoderPositionThreshold * kEncoderPositionToUnits;
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

    @Override
    protected <T> void addFollower(T follower) {
        if (follower instanceof CANSparkMax) {
            CANSparkMax cFollower = (CANSparkMax) follower;
            cFollower.follow(master);
            cFollower.setInverted(master.getInverted());
        }
    }
}
