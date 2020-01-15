package frc.team3647Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import lib.DriveSignal;
import lib.drivers.SparkMaxUtil;


public class Drivetrain implements Subsystem {

    private static final double kDt = .02;
    private CANSparkMax leftMaster;
    private CANSparkMax rightMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightSlave;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private double[] m_leftVelocityPIDArr;
    private double[] m_rightVelocityPIDArr;

    private CANPIDController m_leftVelocityPID;
    private CANPIDController m_rightVelocityPID;

    private int m_maxCurrent;
    private int m_maxStallCurrent;

    private boolean initialized = false;
    private DrivetrainControlType controlType;
    private PeriodicIO periodicIO = new PeriodicIO();

    private SimpleMotorFeedforward feedForwardCalculator;
    private double kS;
    private double kV;
    private double kA;

    private double m_timeStamp;

    private double kEncoderValueToMeters;
    private double kEncoderVelocityToMetersPerSecond;

    Drivetrain(int leftMasterPin, int rightMasterPin, int leftSlavePin, int rightSlavePin,
            double[] leftVelocityPIDArr, double[] rightVelocityPIDArr,
            double[] leftFeedForwardConstants, double[] rightFeedForwardConstants, int maxCurrent,
            int maxStallCurrent, double kEncoderValueToMeters,
            double kEncoderVelocityToMetersPerSecond) {
        leftMaster = new CANSparkMax(leftMasterPin, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMaster = new CANSparkMax(rightMasterPin, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftSlave = new CANSparkMax(leftSlavePin, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightSlave = new CANSparkMax(rightSlavePin, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftEncoder = new CANEncoder(leftMaster);
        rightEncoder = new CANEncoder(rightMaster);

        m_leftVelocityPID = new CANPIDController(leftMaster);
        m_rightVelocityPID = new CANPIDController(rightMaster);

        m_leftVelocityPIDArr = leftVelocityPIDArr;
        m_rightVelocityPIDArr = rightVelocityPIDArr;

        m_maxCurrent = maxCurrent;
        m_maxStallCurrent = maxStallCurrent;

        feedForwardCalculator = new SimpleMotorFeedforward(kS, kV, kA);

        this.kEncoderValueToMeters = kEncoderValueToMeters;
        this.kEncoderVelocityToMetersPerSecond = kEncoderVelocityToMetersPerSecond;

        controlType = DrivetrainControlType.OPENLOOP;

        initialized = false;
    }

    public static class PeriodicIO {
        // inputs
        public double leftEncoderVelocity;
        public double rightEncoderVelocity;

        public double leftEncoderValue;
        public double rightEncoderValue;
        public double heading;

        public double leftAcceleration;
        public double rightAcceleration;

        // outputs
        public double leftOutput;
        public double rightOutput;

        /**
         * in volts
         */
        public double leftFeedForward;
        /**
         * in volts
         */
        public double rightFeedForward;

        /**
         * in meters per second
         */
        public double prevDesiredThrottle;

        /**
         * in meters per second
         */
        public double prevDesiredTurn;
    }

    @Override
    public synchronized void init() {
        SparkMaxUtil.checkError(leftSlave.follow(leftMaster),
                m_timeStamp + " Couldn't follow left master");
        SparkMaxUtil.checkError(rightSlave.follow(rightMaster),
                m_timeStamp + " Coudln't follow right master");


        rightMaster.setInverted(true);
        configPID(m_leftVelocityPID, "Left Master", m_leftVelocityPIDArr, 0);
        configPID(m_rightVelocityPID, "Right Master", m_rightVelocityPIDArr, 0);

        SparkMaxUtil.checkError(leftMaster.setSmartCurrentLimit(m_maxStallCurrent, m_maxCurrent),
                m_timeStamp + " Coudln't set current limiting for left master");
        SparkMaxUtil.checkError(rightMaster.setSmartCurrentLimit(m_maxStallCurrent, m_maxCurrent),
                m_timeStamp + " Coudln't set current limiting for right master");
        SparkMaxUtil.checkError(leftSlave.setSmartCurrentLimit(m_maxStallCurrent, m_maxCurrent),
                m_timeStamp + " Coudln't set current limiting for left slave");
        SparkMaxUtil.checkError(rightSlave.setSmartCurrentLimit(m_maxStallCurrent, m_maxCurrent),
                m_timeStamp + " Coudln't set current limiting for right slave");

        setToBrake();
        initialized = true;
    }



    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.leftEncoderValue = leftEncoder.getPosition();
        periodicIO.rightEncoderValue = rightEncoder.getPosition();
        periodicIO.leftEncoderVelocity = leftEncoder.getVelocity();
        periodicIO.rightEncoderVelocity = rightEncoder.getVelocity();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (DrivetrainControlType.VELOCITY.equals(controlType)) {
            SparkMaxUtil.checkError(
                    m_leftVelocityPID.setReference(periodicIO.leftOutput, ControlType.kVelocity, 0,
                            periodicIO.leftFeedForward),
                    m_timeStamp + " left sparkmax couldn't follow velocity "
                            + periodicIO.leftOutput);

            SparkMaxUtil.checkError(
                    m_rightVelocityPID.setReference(periodicIO.rightOutput, ControlType.kVelocity,
                            0, periodicIO.rightFeedForward),
                    m_timeStamp + " right sparkmax couldn't follow velocity "
                            + periodicIO.rightOutput);

        } else {
            leftMaster.setVoltage(periodicIO.leftOutput);
            rightMaster.setVoltage(periodicIO.rightOutput);
        }
    }

    @Override
    public synchronized void periodic(double timestamp) {
        m_timeStamp = timestamp;
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

    }

    public synchronized void arcadeDrive(double throttle, double turn, boolean scaleInputs) {
        throttle = limit(throttle) * 3.0; // map -1 to 1 to m/s
        turn = limit(turn) * 3.0; // map -1 to 1 to m/s

        double tempThrottle = throttle;
        double tempTurn = turn;

        throttle = feedForwardCalculator.calculate(throttle,
                (throttle - periodicIO.prevDesiredThrottle) / kDt);
        turn = feedForwardCalculator.calculate(turn, (turn - periodicIO.prevDesiredTurn) / kDt);

        periodicIO.prevDesiredThrottle = tempThrottle;
        periodicIO.prevDesiredTurn = tempTurn;

        double leftMotorOutput;
        double rightMotorOutput;
        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

        if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            } else {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            }
        }

        if (scaleInputs) {
            leftMotorOutput = limit(leftMotorOutput) * .6;
            rightMotorOutput = limit(rightMotorOutput) * .6;
        }

        setOpenLoop(new DriveSignal(leftMotorOutput, rightMotorOutput));
    }

    public synchronized void setOpenLoop(DriveSignal driveSignal) {
        periodicIO.leftOutput = driveSignal.getLeft();
        periodicIO.rightOutput = driveSignal.getRight();
        controlType = DrivetrainControlType.OPENLOOP;
    }

    /**
     * @param driveSignal in meters per second
     */
    public synchronized void setVelocity(DriveSignal driveSignal) {
        periodicIO.leftOutput = driveSignal.getLeft() / kEncoderVelocityToMetersPerSecond;
        periodicIO.leftFeedForward = feedForwardCalculator.calculate(periodicIO.leftOutput,
                (periodicIO.leftOutput - getLeftVelocity() / kDt));


        periodicIO.rightOutput = driveSignal.getRight() / kEncoderVelocityToMetersPerSecond;
        periodicIO.leftFeedForward = feedForwardCalculator.calculate(periodicIO.rightOutput,
                (periodicIO.rightOutput - getRightVelocity()) / kDt);


        controlType = DrivetrainControlType.VELOCITY;
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
        SparkMaxUtil.checkError(leftEncoder.setPosition(0),
                m_timeStamp + " Couldn't reset left encoder");
        SparkMaxUtil.checkError(rightEncoder.setPosition(0),
                m_timeStamp + " Couldn't reset right encoder");;
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

    private enum DrivetrainControlType {
        OPENLOOP, VELOCITY
    }

    private synchronized void configPID(CANPIDController controller, String name, double[] PIDArr,
            int slot) {
        SparkMaxUtil.checkError(controller.setP(PIDArr[0], slot),
                m_timeStamp + " Couldn't set P for " + name);
        SparkMaxUtil.checkError(controller.setI(PIDArr[1], slot),
                m_timeStamp + " Couldn't set I for " + name);
        SparkMaxUtil.checkError(controller.setD(PIDArr[2], slot),
                m_timeStamp + " Couldn't set D for " + name);
    }

    /**
     * @return left position in meters
     */
    public double getLeftEncoderPosition() {
        return periodicIO.leftEncoderValue * kEncoderValueToMeters;
    }


    /**
     * @return right encoder position in meters
     */
    public double getRightEncoderValue() {
        return periodicIO.rightEncoderValue * kEncoderValueToMeters;
    }

    public double getLeftVelocity() {
        return periodicIO.leftEncoderVelocity * kEncoderVelocityToMetersPerSecond;
    }

    public double getRightVelocity() {
        return periodicIO.rightEncoderVelocity * kEncoderVelocityToMetersPerSecond;
    }

    public boolean hasInitialized() {
        return initialized;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
