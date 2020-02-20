package frc.team3647Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import lib.drivers.ClosedLoopFactory;
import lib.drivers.SparkMaxFactory;
import lib.drivers.SparkMaxUtil;
import lib.wpi.HALMethods;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.SparkMaxFactory.Configuration;

/**
 * Add you own bounds.
 */
public abstract class SparkMaxSubsystem implements PeriodicSubsystem {

    public static double kDt = .01;

    private CANSparkMax master;
    private CANEncoder encoder;
    private CANPIDController pidController;
    private ControlType controlType = ControlType.kDutyCycle;
    private Configuration m_masterConfig;
    private ClosedLoopConfig m_pidConfig;
    private SimpleMotorFeedforward feedforward;

    public static class PeriodicIO {
        // inputs
        public double position;
        public double velocity;
        public double prevVelocity;
        // outputs
        /** In Volts */
        public double feedforward;
        public double demand;
    }

    private PeriodicIO periodicIO = new PeriodicIO();

    protected SparkMaxSubsystem(Configuration masterConfig, ClosedLoopConfig pidConfig) {
        boolean error = false;
        if (masterConfig == null) {
            HALMethods.sendDSError("Master config sparkmax Subsystem " + getName() + " was null");
            error = true;
        }

        if (pidConfig == null) {
            HALMethods.sendDSError("pid config sparkmax subsystem " + getName() + " was null");
            error = true;
        }

        if (error) {
            throw new IllegalArgumentException("either master config or pid config were null");
        }
        m_masterConfig = masterConfig;
        m_pidConfig = pidConfig;
        master = SparkMaxFactory.createSparkMax(m_masterConfig);
        encoder = new CANEncoder(master);
        pidController = ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
        feedforward = new SimpleMotorFeedforward(pidConfig.kS, pidConfig.kV, pidConfig.kA);
    }

    @Override
    public void init() {
        try {
            setToBrake();
            ClosedLoopFactory.configSparkMaxPIDController(pidController, master, encoder, m_pidConfig, 0);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            master = SparkMaxFactory.createSparkMax(m_masterConfig);
            encoder = master.getEncoder();
            pidController = ClosedLoopFactory.createSparkMaxPIDController(master, encoder, m_pidConfig, 0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = encoder.getPosition() * m_pidConfig.kEncoderTicksToUnits;
        periodicIO.velocity = encoder.getVelocity() * m_pidConfig.kEncoderVelocityToRPM;
    }

    @Override
    public void writePeriodicOutputs() {
        try {
            setSparkMax(controlType, periodicIO.demand, periodicIO.feedforward);
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            controlType = ControlType.kDutyCycle;
            periodicIO = new PeriodicIO();
        }
    }

    @Override
    public void end() {
        setOpenloop(0);
        periodicIO.demand = 0;
        periodicIO.feedforward = 0;
        controlType = ControlType.kDutyCycle;
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
        periodicIO.feedforward = feedforward.calculate(velocity / 60);
        controlType = ControlType.kVelocity;
    }

    public void setOpenloop(double demand) {
        periodicIO.demand = demand;
        periodicIO.feedforward = 0;
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

    public boolean getTargetPosition() {
        if (controlType == ControlType.kPosition || controlType == ControlType.kSmartMotion) {
            return reachedPosition(periodicIO.demand);
        }
        return false;
    }

    public boolean reachedVelocity(double velocity) {
        System.out.println(getName() + " velociy requested to check" + velocity);
        System.out.println(getName() + " current velocity " + getVelocity());
        return velocity < getVelocity() + m_pidConfig.velocityThreshold
                && velocity > getVelocity() - m_pidConfig.velocityThreshold;
    }

    public boolean reachedTargetVelocity() {
        if (controlType == ControlType.kVelocity) {
            return reachedVelocity(periodicIO.demand * m_pidConfig.kEncoderVelocityToRPM);
        }
        return false;
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
        // if the feed is outside -12 to 12, adjust to -12 to 12
        feedForward = Math.abs(feedForward) > 12 ? 12 * Math.signum(feedForward) : feedForward;
        if (controlType == ControlType.kDutyCycle) {
            master.set(demand);
        } else {
            pidController.setReference(demand, controlType, 0, feedForward);
        }
    }

    protected CANSparkMax addFollower(SparkMaxFactory.Configuration config, boolean isInvertedFromMaster) {
        CANSparkMax follower = SparkMaxFactory.createSparkMaxFollower(master, config, isInvertedFromMaster);
        follower.follow(master, isInvertedFromMaster);
        return follower;
    }

    protected CANSparkMax getMaster() {
        return master;
    }
}
