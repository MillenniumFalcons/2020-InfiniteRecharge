package frc.team3647Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import lib.drivers.ClosedLoopFactory;
import lib.drivers.SparkMaxFactory;
import lib.drivers.SparkMaxUtil;
import lib.wpi.HALMethods;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.SparkMaxFactory.Configuration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;;

public abstract class SparkMaxSubsystem extends SubsystemBase implements PeriodicSubsystem{

    private CANSparkMax master;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private ControlType controlType = ControlType.kDutyCycle;
    private Configuration m_masterConfig;
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

    protected SparkMaxSubsystem(Configuration masterConfig, ClosedLoopConfig pidConfig) {
        m_masterConfig = masterConfig;
        m_pidConfig = pidConfig;
        master = SparkMaxFactory.createSparkMax(m_masterConfig);
        encoder = master.getEncoder();
        pidController =
                ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
    }

    public void init() {
        try {
            setToBrake();
            ClosedLoopFactory.configSparkMaxPIDController(pidController, master, encoder,
                    m_pidConfig, 0);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            encoder = master.getEncoder();
            pidController =
                    ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
        }
    }

    public void readPeriodicInputs() {
        periodicIO.position = encoder.getPosition() * m_pidConfig.kEncoderTicksToUnits;
        periodicIO.velocity = encoder.getVelocity() * m_pidConfig.kEncoderVelocityToRPM;
    }

    public void writePeriodicOutputs() {
        setSparkMax(controlType, periodicIO.demand, periodicIO.feedforward);
    }

    public void end() {
        setOpenloop(0);
        periodicIO.feedforward = 0;
    }


    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    protected void setEncoderPosition(double newPosition) {
        SparkMaxUtil.checkError(encoder.setPosition(newPosition / m_pidConfig.kEncoderTicksToUnits),
                "couldn't change encoder value on " + getName() + "'s' sparkmax");
    }

    protected void setPosition(double refrencePt) {
        periodicIO.demand = refrencePt / m_pidConfig.kEncoderTicksToUnits;
        controlType = ControlType.kSmartMotion;
    }

    protected void setVelocity(double velocity) {
        periodicIO.demand = velocity / m_pidConfig.kEncoderVelocityToRPM;
        controlType = ControlType.kSmartMotion;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        controlType = ControlType.kDutyCycle;
    }

    public double getPosition() {
        return periodicIO.position;
    }

    public double getVelocity() {
        return periodicIO.velocity;
    }

    /**
     * @param position position to check in real world units
     */
    protected boolean reachedPosition(double position) {
        return position < getPosition() + m_pidConfig.positionThreshold
                && position > getPosition() - m_pidConfig.positionThreshold;
    }

    public void setToCoast() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kCoast),
                    " Couldn't set to coast " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            HALMethods.sendDSError(e.toString());
        }
    }

    public void setToBrake() {
        try {
            SparkMaxUtil.checkError(master.setIdleMode(IdleMode.kBrake),
                    " Couldn't set to brake " + getName() + " sparkmax");
        } catch (NullPointerException e) {
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            HALMethods.sendDSError(e.toString());
        }
    }

    private void setSparkMax(ControlType controlType, double demand, double feedForward) {
        pidController.setReference(demand, controlType, 0, feedForward);
    }

    protected <T> void addFollower(T follower) {
        if (follower instanceof CANSparkMax) {
            CANSparkMax cFollower = (CANSparkMax) follower;
            cFollower.follow(master);
            cFollower.setInverted(master.getInverted());
        }
    }
}
