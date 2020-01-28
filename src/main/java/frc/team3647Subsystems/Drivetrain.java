package frc.team3647Subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MathUtil;
import lib.DriveSignal;
import lib.drivers.ClosedLoopFactory;
import lib.drivers.SparkMaxFactory;
import lib.drivers.SparkMaxUtil;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.SparkMaxFactory.Configuration;
import lib.wpi.HALMethods;
import lib.wpi.Solenoid;
import lib.wpi.Timer;



public class Drivetrain implements PeriodicSubsystem {

    private static int constructCount = 0;
    private static final double kDt = .02;
    private CANSparkMax leftMaster;
    private CANSparkMax rightMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightSlave;

    private Configuration m_leftMasterConfig;
    private Configuration m_rightMasterConfig;

    private ClosedLoopConfig m_leftPIDConfig;
    private ClosedLoopConfig m_rightPIDConfig;
    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private CANPIDController m_leftVelocityPID;
    private CANPIDController m_rightVelocityPID;

    private boolean initialized = false;
    private PeriodicIO periodicIO = new PeriodicIO();

    private SimpleMotorFeedforward feedforward;

    private final double kEncoderVelocityToMetersPerSecond;

    private ControlType controlType = ControlType.kDutyCycle;
    private double m_timeStamp;
    private double m_maxOutput;

    private final DifferentialDriveOdometry m_odometry;

    private PigeonIMU m_gyro;

    private Solenoid leftShifter;
    private Solenoid rightShifter;

    private boolean shifted;

    public Drivetrain(Configuration leftMasterConfig, Configuration rightMasterConfig,
            Configuration leftSlaveConfig, Configuration rightSlaveConfig,
            ClosedLoopConfig leftMasterPIDConfig, ClosedLoopConfig rightMasterPIDConfig,
            int leftShifterPin, int rightShifterPin, double kWheelDiameterMeters) {
        if (constructCount > 0) {
            throw new UnsupportedOperationException("Drivetrain was already initialized once");
        }
        m_leftMasterConfig = leftMasterConfig;
        m_rightMasterConfig = rightMasterConfig;

        m_leftPIDConfig = leftMasterPIDConfig;
        m_rightPIDConfig = rightMasterPIDConfig;

        leftMaster = SparkMaxFactory.createSparkMax(m_leftMasterConfig);
        rightMaster = SparkMaxFactory.createSparkMax(m_rightMasterConfig);

        leftSlave = SparkMaxFactory.createSparkMaxFollower(leftMaster, leftSlaveConfig);
        leftSlave = SparkMaxFactory.createSparkMaxFollower(rightMaster, rightSlaveConfig);

        leftEncoder = leftMaster.getEncoder();
        rightEncoder = rightMaster.getEncoder();

        m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster, leftEncoder,
                m_leftPIDConfig, 0);
        m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                rightEncoder, m_rightPIDConfig, 0);

        kEncoderVelocityToMetersPerSecond =
                m_leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameterMeters * Math.PI;
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        constructCount++;
        shifted = false;
    }

    public static class PeriodicIO {
        // inputs
        /** Meters per second */
        public double leftVelocity;
        /** Meters per second */
        public double rightVelocity;

        public double ypr[];

        /** Meters */
        public double leftPosition;
        /** Meters */
        public double rightPosition;
        /** Degrees -180 to 180 */
        public double heading;

        // outputs
        public double leftOutput;
        public double rightOutput;

        /** Meters Per second */
        public double prevLeftDesiredVelocity;
        /** Meters Per Second */
        public double prevRightDesiredVelocity;

        /** Volts */
        public double leftFeedForward;
        /** Volts */
        public double rightFeedForward;
    }

    @Override
    public synchronized void init() {
        setToBrake();
        initialized = true;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        try {
            periodicIO.leftPosition =
                    leftEncoder.getPosition() * m_leftPIDConfig.kEncoderTicksToUnits;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructLeftEncoder();
            periodicIO.leftPosition =
                    leftEncoder.getPosition() * m_leftPIDConfig.kEncoderTicksToUnits;
        }

        try {
            periodicIO.rightPosition =
                    rightEncoder.getPosition() * m_rightPIDConfig.kEncoderTicksToUnits;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructRightEncoder();
            periodicIO.rightPosition =
                    rightEncoder.getPosition() * m_rightPIDConfig.kEncoderTicksToUnits;
        }

        try {
            periodicIO.leftVelocity = leftEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructLeftEncoder();
            periodicIO.leftVelocity = leftEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        }

        try {
            periodicIO.rightVelocity =
                    rightEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        } catch (NullPointerException e) {
            HALMethods.sendDSError(e.toString());
            reconstructRightEncoder();
            periodicIO.rightVelocity =
                    rightEncoder.getVelocity() * kEncoderVelocityToMetersPerSecond;
        }

        m_gyro.getYawPitchRoll(periodicIO.ypr);
        periodicIO.heading = Math.IEEEremainder(periodicIO.ypr[0], 360);
    }

    private void reconstructLeftEncoder() {
        leftEncoder = leftMaster.getEncoder();
        m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster, leftEncoder,
                m_leftPIDConfig, 0);
    }

    private void reconstructRightEncoder() {
        rightEncoder = leftMaster.getEncoder();
        m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                rightEncoder, m_rightPIDConfig, 0);
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (shifted && periodicIO.leftOutput != periodicIO.rightOutput) {
            HALMethods.sendDSError("Left motor outout: " + periodicIO.leftOutput
                    + " and right output: " + periodicIO.rightOutput + " are not equal!!");
            HALMethods.sendDSError("Setting drivetrain to stop!!");
            periodicIO.leftOutput = 0;
            periodicIO.rightOutput = 0;
            periodicIO.leftFeedForward = 0;
            periodicIO.rightFeedForward = 0;
        }

        try {
            m_leftVelocityPID.setReference(periodicIO.leftOutput, controlType, 0,
                    periodicIO.leftFeedForward);
        } catch (NullPointerException e) {
            m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster,
                    leftEncoder, m_leftPIDConfig, 0);
            m_leftVelocityPID.setReference(periodicIO.leftOutput, controlType, 0,
                    periodicIO.leftFeedForward);
            HALMethods.sendDSError(e.toString());
        }

        try {
            m_rightVelocityPID.setReference(periodicIO.rightOutput, controlType, 0,
                    periodicIO.rightFeedForward);
        } catch (NullPointerException e) {
            m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                    rightEncoder, m_rightPIDConfig, 0);
            m_rightVelocityPID.setReference(periodicIO.rightOutput, controlType, 0,
                    periodicIO.rightFeedForward);
            HALMethods.sendDSError(e.toString());
        }
    }

    @Override
    public void periodic() {
        PeriodicSubsystem.super.periodic();
        m_timeStamp = Timer.getFPGATimestamp();
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), periodicIO.leftPosition,
                periodicIO.rightPosition);
    }

    @Override
    public synchronized void end() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();
        leftSlave.stopMotor();
        rightSlave.stopMotor();
        periodicIO.leftOutput = 0;
        periodicIO.rightOutput = 0;
        periodicIO.leftFeedForward = 0;
        periodicIO.rightFeedForward = 0;
        controlType = ControlType.kDutyCycle;

    }

    public synchronized void setOpenLoop(DriveSignal driveSignal) {
        if (driveSignal != null) {
            periodicIO.leftOutput = driveSignal.getLeft();
            periodicIO.rightOutput = driveSignal.getRight();
            periodicIO.leftFeedForward = 0;
            periodicIO.rightFeedForward = 0;
            controlType = ControlType.kDutyCycle;
        } else {
            end();
            HALMethods.sendDSError("DriveSignal in setOpenLoop was null");
        }
    }

    private void setShifters(boolean on) {
        leftShifter.set(on);
        rightShifter.set(on);
        shifted = on;
    }

    public void shift() {
        setShifters(true);
    }

    public void unShift() {
        setShifters(false);
    }

    /**
     * @param driveSignal in meters per second
     */
    public synchronized void setVelocity(DriveSignal driveSignal) {
        if (driveSignal != null) {
            if (!isShifted()) {
                periodicIO.leftFeedForward = feedforward.calculate(driveSignal.getLeft(),
                        (driveSignal.getLeft() - periodicIO.leftVelocity) / .02);
                periodicIO.rightFeedForward = feedforward.calculate(driveSignal.getRight(),
                        (driveSignal.getRight() - periodicIO.rightVelocity) / .02);

                periodicIO.leftOutput = driveSignal.getLeft() / kEncoderVelocityToMetersPerSecond;
                periodicIO.rightOutput = driveSignal.getRight() / kEncoderVelocityToMetersPerSecond;
                controlType = ControlType.kVelocity;
            }
        } else {
            HALMethods.sendDSError("Drive signal in setVelocity(DriveSignal) was null");
            end();
        }
    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean scaleInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .09);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .09);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        if (scaleInputs) {
            m_maxOutput = .7;
        }
        double currentLeftDesiredVelocity = MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput
                * m_leftPIDConfig.maxVelocity;
        double currentRightDesiredVelocity = MathUtil.clamp(rightMotorOutput, -1.0, 1.0)
                * m_maxOutput * m_rightPIDConfig.maxVelocity;

        double leftVoltage = feedforward.calculate(currentLeftDesiredVelocity,
                (currentLeftDesiredVelocity - periodicIO.prevLeftDesiredVelocity) / kDt);
        double rightVoltage = feedforward.calculate(currentRightDesiredVelocity,
                (currentRightDesiredVelocity - periodicIO.prevRightDesiredVelocity) / kDt);

        if (shifted) {
            setOpenLoop(new DriveSignal(xSpeed, xSpeed));
        } else {
            setOpenLoop(new DriveSignal(leftVoltage / m_leftMasterConfig.nominalVoltage,
                    rightVoltage / m_rightMasterConfig.nominalVoltage));
        }

        periodicIO.prevLeftDesiredVelocity = leftMotorOutput * m_leftPIDConfig.maxVelocity;
        periodicIO.prevRightDesiredVelocity = rightMotorOutput * m_rightPIDConfig.maxVelocity;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public synchronized void setToCoast() {
        SparkMaxUtil.checkError(leftMaster.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set left master to Coast mode");
        SparkMaxUtil.checkError(rightMaster.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set right master to Coast mode");

        SparkMaxUtil.checkError(leftSlave.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set left slave to Coast mode");
        SparkMaxUtil.checkError(rightSlave.setIdleMode(IdleMode.kCoast),
                m_timeStamp + " Coudln't set right slave to Coast mode");
    }


    public synchronized void setToBrake() {
        SparkMaxUtil.checkError(leftMaster.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set left master to brake mode");
        SparkMaxUtil.checkError(rightMaster.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set right master to brake mode");

        SparkMaxUtil.checkError(leftSlave.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set left slave to brake mode");
        SparkMaxUtil.checkError(rightSlave.setIdleMode(IdleMode.kBrake),
                m_timeStamp + " Coudln't set right slave to brake mode");
    }

    public synchronized void resetEncoders() {
        try {
            SparkMaxUtil.checkError(leftEncoder.setPosition(0),
                    m_timeStamp + " Couldn't reset left encoder");
        } catch (NullPointerException e) {
            leftEncoder = leftMaster.getEncoder();
            m_leftVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(leftMaster,
                    leftEncoder, m_leftPIDConfig, 0);
            HALMethods.sendDSError(e.toString());
        }

        try {
            SparkMaxUtil.checkError(rightEncoder.setPosition(0),
                    m_timeStamp + " Couldn't reset right encoder");
        } catch (NullPointerException e) {
            rightEncoder = rightMaster.getEncoder();
            m_rightVelocityPID = ClosedLoopFactory.createSparkMaxPIDController(rightMaster,
                    rightEncoder, m_rightPIDConfig, 0);
            HALMethods.sendDSError(e.toString());
        }
        periodicIO = new PeriodicIO();
    }

    private double limit(double value) {
        if (value > 1) {
            value = 1;
        } else if (value < -1) {
            value = -1;
        }
        return value;
    }

    /**
     * @return left position in meters
     */
    public double getLeftPosition() {
        return periodicIO.leftPosition;
    }

    /**
     * @return right encoder position in meters
     */
    public double getRightPosition() {
        return periodicIO.rightPosition;
    }

    public double getLeftVelocity() {
        return periodicIO.leftVelocity;
    }

    public double getRightVelocity() {
        return periodicIO.rightVelocity;
    }

    public boolean hasInitialized() {
        return initialized;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(periodicIO.leftVelocity, periodicIO.rightVelocity);
    }

    public boolean isShifted() {
        return shifted;
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (periodicIO.leftPosition + periodicIO.rightPosition) / 2.0;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        m_gyro.setYaw(0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
