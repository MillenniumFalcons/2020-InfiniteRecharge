package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.team3647Subsystems.VisionController;
import lib.drivers.SparkMaxFactory;
import lib.drivers.TalonSRXFactory;
import lib.drivers.VictorSPXFactory;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.team3647Utils.RGB;

public class Constants {

    public static class cDrivetrain {
        // CAN id
        public static final int leftMasterPin = 1;
        public static final int leftSlavePin = 2;
        public static final int rightMasterPin = 3;
        public static final int rightSlavePin = 4;
        public static final int stallCurrent = 35;
        public static final int maxCurrent = 60;

        // pcm pin
        public static final int shifterPin = 2;

        // meters (6inches)
        public static final double kWheelDiameter = .1524;

        // volts
        public static final double kS = 1.3;
        public static final double kV = 12.0 / 3.2;
        public static final double kA = 0.195;

        public static final double kTrackwidthMeters = 0.7112;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3.2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

        public static final double maxVoltage = 10.0;
        public static final double gearboxReduction = 9.0 / 40.0 * 24.0 / 42.0;

        public static final double neoRotationsToMeters =
                gearboxReduction * kWheelDiameter * Math.PI;

        public static final SparkMaxFactory.Configuration leftMasterConfig =
                new SparkMaxFactory.Configuration(leftMasterPin, false)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration rightMasterConfig =
                new SparkMaxFactory.Configuration(rightMasterPin, true)
                        .currentLimiting(true, maxCurrent, stallCurrent).idleMode(IdleMode.kBrake)
                        .voltageCompensation(true, 12.0);

        public static final SparkMaxFactory.Configuration leftSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(leftMasterConfig, leftSlavePin);

        public static final SparkMaxFactory.Configuration rightSlaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(rightMasterConfig, rightSlavePin);

        public static final ClosedLoopConfig leftMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters);
        public static final ClosedLoopConfig rightMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters);

    }

    public static class cIntake {
        public static final int outerPistonsPin = 0;
        public static final int innerPistonsPin = 1;
        public static final int intakeMotorPin = 8;

        public static final boolean inverted = false;
        public static TalonSRXFactory.Configuration intakeMotorConfig =
                new TalonSRXFactory.Configuration(intakeMotorPin, inverted);
    }

    public static class cPPSpinner {
        public static final RGB red = new RGB(new double[] {.51, .35, .14});
        public static final RGB green = new RGB(new double[] {.15, .59, .25});
        public static final RGB blue = new RGB(new double[] {.12, .42, .45});
        public static final RGB yellow = new RGB(new double[] {.32, .56, .12});
        // public static final RGB test = new RGB(new double[] {});
        public static final double colorThreshold = 0.05; // percentage
    }

    public static class cVisionController {
        // 8ft in meters
        public static final double kGoalHeight = 2.438;

        // 30inches in meters
        public static final double kCameraHeight = 0.762;

        // can be either 75 or 56 degrees depending on the lens setting used
        public static final double kFOV = 75;

        /** in micro meters */
        public static final double kSensorHeight = 2952;
        /** in micro meters */
        public static final double kSensorWidth = 3984;

        /** micrometers per pixel */
        public static final double kPixelSize = 6;

        /** in degrees */
        public static final double camAngle = 30;

        public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

        public static final VisionController.CamConstants camConstants =
                new VisionController.CamConstants(kGoalHeight, kCameraHeight, camAngle);
    }

    public static class cTurret {
        public static final int masterPin = 17;
        public static final boolean inverted = false;
        public static final double nominalVoltage = 10.0;
        public static final int limitSwitchPin = 0;
        public static final boolean sensorInverted = true;

        public static final double kMinRotationDeg = -180;
        public static final double kMaxRotationDeg = 170;
        // amps
        public static final int peakCurrent = 40;
        // milis
        public static final int peakCurrentDuration = 1000;
        // amps
        public static final int continuousCurrent = 20;

        public static final int reductionFromEncoder = 1;

        public static final double encoderTicksToUnits =
                (reductionFromEncoder / magEncoderTicksPerRev) * 360.0;
        public static final double encoderVelocityToRPM =
                reductionFromEncoder / magEncoderTicksPerRev * 10 * 60;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double forwardDeg = 0;
        public static final double backwardDeg = 180;
        public static final double leftDeg = 90;
        public static final double rightDeg = -90;

        public static final TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);

        public static final ClosedLoopConfig pidConfig = new ClosedLoopConfig()
                .encoderTicksToUnits(encoderTicksToUnits).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM).positionThreshold(.5)
                .sensorInverted(sensorInverted).configPID(kP, kI, kD);
    }

    public static class cIndexer {
        public static final int funnelPin = 22;
        public static final int tunnelPin = 23;
        public static final int rollersPin = 24;
        public static final int bannerSensorPin = 1;

        public static boolean funnelInverted = true;
        public static boolean tunnelInverted = false;
        public static boolean rollersInverted = true;

        public static VictorSPXFactory.Configuration funnelConfig =
                new VictorSPXFactory.Configuration(funnelPin).setInverted(funnelInverted);

        public static TalonSRXFactory.Configuration tunnelConfig =
                new TalonSRXFactory.Configuration(tunnelPin, tunnelInverted)
                        .neutralMode(NeutralMode.Coast);
        public static VictorSPXFactory.Configuration rollersConfig =
                new VictorSPXFactory.Configuration(rollersPin).setInverted(funnelInverted);

    }

    public static class cKickerWheel {
        public static final int masterPin = 15;
        public static final boolean inverted = false;
        public static final int peakCurrent = 60;
        public static final int peakCurrentDuration = 1000;
        public static final int continuousCurrent = 20;
        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double encoderTicksToUnits = 1 / magEncoderTicksPerRev;
        public static final double encoderVelocityToRPM = encoderTicksToUnits * 10 * 60;

        public static final double visionDistanceToRPM = 1;

        public static TalonSRXFactory.Configuration masterConfig =
                new TalonSRXFactory.Configuration(masterPin, inverted);
        // .neutralMode(NeutralMode.Brake).voltageCompensation(true, nominalVoltage)
        // .currentLimiting(true, peakCurrent, peakCurrentDuration, continuousCurrent);
        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD);
        // .configFeedForward(kS, kV, kA).encoderTicksToUnits(encoderTicksToUnits)
        // .encoderVelocityToRPM(encoderVelocityToRPM)
        // .encoderAccelerationToUnits(encoderVelocityToRPM);
    }

    public static class cFlywheel {
        public static final int masterPin = 11;
        public static final int slavePin = 12;
        public static final boolean masterInverted = false;
        public static final boolean slaveInverted = false;

        public static final int maxFreeSpeedCurrent = 60;
        public static final int maxStallCurrent = 40;

        public static final double nominalVoltage = 12.0;
        public static final NeutralMode neutralMode = NeutralMode.Brake;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double kGearboxReduction = 2;
        public static final double encoderTicksToUnits = 42;
        public static final double encoderVelocityToRPM = 2;

        public static final double visionDistanceToRPM = 1;

        public static SparkMaxFactory.Configuration masterConfig =
                new SparkMaxFactory.Configuration(masterPin, masterInverted)
                        .idleMode(IdleMode.kBrake).voltageCompensation(true, nominalVoltage)
                        .currentLimiting(true, maxFreeSpeedCurrent, maxStallCurrent);
        public static SparkMaxFactory.Configuration slaveConfig =
                SparkMaxFactory.Configuration.mirrorWithCANID(masterConfig, slavePin);

        public static ClosedLoopConfig pidConfig = new ClosedLoopConfig().configPID(kP, kI, kD)
                .configFeedForward(kS, kV, kA).encoderTicksToUnits(encoderTicksToUnits)
                .encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double calculateRPM(double distance) {
            return distance;
        }
    }

    /**
     * In order to make the class not be able to be an object
     */
    private Constants() {
    }

    public static double magEncoderTicksPerRev = 4096.0;
}
