package team3647.frc2020.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;
import team3647.frc2020.subsystems.VisionController;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.drivers.SparkMaxFactory;
import team3647.lib.drivers.TalonSRXFactory;
import team3647.lib.drivers.VictorSPXFactory;
import team3647.lib.util.RGB;

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
        public static final double kS = 0.261;
        public static final double kV = 2.62;
        // public static final double kA = 0.353;
        public static final double kA = 0.15;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kTrackwidthMeters = 1.108081713274498;
        public static final double kTrackWidthMeters2 = .7239849047751171;
        public static final DifferentialDriveKinematics kDriveKinematics =
                new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;

        public static final double maxVoltage = 11.0;
        public static final double gearboxReduction = 9.0 / 42.0 * 24.0 / 50.0;

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
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);
        public static final ClosedLoopConfig rightMasterPIDConfig = new ClosedLoopConfig()
                .encoderVelocityToRPM(gearboxReduction).encoderTicksToUnits(neoRotationsToMeters)
                .maxVelocity(kMaxSpeedMetersPerSecond).configPID(kP, kI, kD);

    }

    public static class cIntake {
        public static final int outerPistonsPin = 4;
        public static final int innerPistonsPin = 3;
        public static final int intakeMotorPin = 8;

        public static final boolean inverted = false;
        public static TalonSRXFactory.Configuration intakeMotorConfig =
                new TalonSRXFactory.Configuration(intakeMotorPin, inverted)
                        .configOpenLoopRampRate(.3);
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
        public static final double kGoalHeight = Units.inchesToMeters(98.25);

        // 30inches in meters
        public static final double kCameraHeight = Units.feetToMeters(3);

        // can be either 75 or 56 degrees depending on the lens setting used
        public static final double kFOV = 75;

        /** in micro meters */
        public static final double kSensorHeight = 2952;
        /** in micro meters */
        public static final double kSensorWidth = 3984;

        /** micrometers per pixel */
        public static final double kPixelSize = 6;

        /** in degrees */
        public static final double camAngle = 23.3;

        public static final double kImageCaptureLatency = 11.0 / 1000.0; // miliseconds

        public static final String camIP = "10.36.47.15";

        public static final VisionController.CamConstants camConstants =
                new VisionController.CamConstants(kGoalHeight, kCameraHeight, camAngle,
                        kImageCaptureLatency);
    }

    public static class cTurret {
        public static final int masterPin = 17;
        public static final boolean inverted = true;
        public static final double nominalVoltage = 10.0;
        public static final int limitSwitchPin = 6;
        public static final boolean sensorInverted = false;

        public static final double kMinRotationDeg = -20;
        public static final double kMaxRotationDeg = 20;

        public static final double kCruiseVelocityRPM = 150;
        public static final double kAccelerationRPMs = 300;
        // amps
        public static final int peakCurrent = 40;
        // milis
        public static final int peakCurrentDuration = 1000;
        // amps
        public static final int continuousCurrent = 20;

        public static final double reductionFromEncoder = 16.0 / 130.0;

        public static final double encoderTicksToUnits =
                (reductionFromEncoder / magEncoderTicksPerRev) * 360.0;
        public static final double encoderVelocityToRPM = encoderTicksToUnits / 360.0 * 10 * 60;

        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 30;

        public static final double kS = 1;
        public static final double kV = 0;
        public static final double kA = 0;

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
                .maxVelocity(50).maxAcceleration(kAccelerationRPMs).sensorInverted(sensorInverted)
                .configPID(kP, kI, kD).configFeedForward(kS, kV, kA);
    }

    public static class cIndexer {
        public static final int funnelPin = 22;
        public static final int tunnelPin = 23;
        public static final int rollersPin = 24;
        public static final int bannerSensorPin = 1;

        public static final int funnelPDPSlot = 10;
        public static final int tunnelPDPSlot = 8;

        public static boolean funnelInverted = true;
        public static boolean tunnelInverted = false;
        public static boolean rollersInverted = false;

        public static VictorSPXFactory.Configuration funnelConfig =
                new VictorSPXFactory.Configuration(funnelPin).setInverted(funnelInverted)
                        .configOpenLoopRampRate(.3).setPDPSlot(10);

        public static VictorSPXFactory.Configuration tunnelConfig =
                new VictorSPXFactory.Configuration(tunnelPin).setInverted(tunnelInverted)
                        .setPDPSlot(8);
        public static VictorSPXFactory.Configuration rollersConfig =
                new VictorSPXFactory.Configuration(rollersPin).setInverted(rollersInverted)
                        .configOpenLoopRampRate(.3);

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
                new TalonSRXFactory.Configuration(masterPin, inverted)
                        .currentLimiting(true, 20, 1, 10).voltageCompensation(true, 12);
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

        public static final double kP = 0.0005;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kS = 0.623;
        public static final double kV = 0.0631;
        // public static final double kV = 0.03;
        public static final double kA = 0.0241;

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
                .velocityThrehsold(75).encoderVelocityToRPM(encoderVelocityToRPM)
                .encoderAccelerationToUnits(encoderVelocityToRPM);

        public static double calculateRPM(double distance) {
            return distance;
        }
    }

    public static class cHood {
        public static int pwmPort = 2;
        public static double minPosition = 0.5;
        public static double maxPosition = 0.9;
    }

    /**
     * In order to make the class not be able to be an object
     */
    private Constants() {
    }

    public static final double magEncoderTicksPerRev = 4096.0;
}
