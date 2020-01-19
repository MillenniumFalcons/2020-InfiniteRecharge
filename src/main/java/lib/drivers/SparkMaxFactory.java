package lib.drivers;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;


/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application. (254)
 */
public class SparkMaxFactory {

    public static class Configuration {
        public final int CANID;
        public final int maxFreeSpeedCurrent;
        public final int maxStallCurrent;
        public final boolean isInverted;
        public final IdleMode idleMode;
        public final boolean voltageCompensation;
        public final double nominalVoltage;

        private Configuration() {
            CANID = 0;
            maxFreeSpeedCurrent = 0;
            maxStallCurrent = 0;
            isInverted = false;
            idleMode = IdleMode.kCoast;
            voltageCompensation = false;
            nominalVoltage = 12;
        }

        public Configuration(int CANID) {
            this.CANID = CANID;
            maxFreeSpeedCurrent = 0;
            maxStallCurrent = 0;
            isInverted = false;
            idleMode = IdleMode.kCoast;
            voltageCompensation = false;
            nominalVoltage = 12;
        }

        public Configuration(int CANID, int maxFreeSpeedCurrent, int maxStallCurrent,
                boolean isInverted, IdleMode idleMode, boolean voltageCompensation,
                double nominalVoltage) {
            this.CANID = CANID;
            this.maxFreeSpeedCurrent = maxFreeSpeedCurrent;
            this.maxStallCurrent = maxStallCurrent;
            this.isInverted = isInverted;

            this.idleMode = idleMode;
            this.voltageCompensation = voltageCompensation;
            this.nominalVoltage = nominalVoltage;
        }

        public static Configuration mirrorWithCANID(Configuration config, int CANID) {
            return new Configuration(CANID, config.maxFreeSpeedCurrent, config.maxStallCurrent,
                    config.isInverted, config.idleMode, config.voltageCompensation,
                    config.nominalVoltage);
        }
    }

    public static final Configuration DEFAULT = new Configuration();

    private static void handleCANError(int id, CANError error, String message) {
        if (error != CANError.kOk) {
            DriverStation.reportError("Could not configure spark id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    public static CANSparkMax createSparkMax(Configuration config) {
        CANSparkMax sparkmax = new CANSparkMax(config.CANID, MotorType.kBrushless);
        handleCANError(config.CANID, sparkmax.restoreFactoryDefaults(), "restore factory defaults");
        handleCANError(config.CANID,
                sparkmax.setSmartCurrentLimit(config.maxStallCurrent, config.maxFreeSpeedCurrent),
                "set current limiting");
        sparkmax.setInverted(config.isInverted);
        handleCANError(config.CANID, sparkmax.setIdleMode(config.idleMode), "set idle mode");
        if (config.voltageCompensation) {
            handleCANError(config.CANID, sparkmax.enableVoltageCompensation(config.nominalVoltage),
                    "set voltage compensation");
        } else {
            handleCANError(config.CANID, sparkmax.disableVoltageCompensation(),
                    "disable voltage compensation");
        }
        return sparkmax;
    }

    public static CANSparkMax createSparkMaxFollower(CANSparkMax master, Configuration config,
            boolean isInvertedFromMaster) {
        CANSparkMax follower = new CANSparkMax(config.CANID, MotorType.kBrushless);
        if (config.voltageCompensation) {
            if (config.voltageCompensation) {
                handleCANError(config.CANID,
                        follower.enableVoltageCompensation(config.nominalVoltage),
                        "set voltage compensation");
            } else {
                handleCANError(config.CANID, follower.disableVoltageCompensation(),
                        "disable voltage compensation");
            }
        }
        handleCANError(config.CANID, follower.follow(master, isInvertedFromMaster), "set follow");
        return follower;
    }

    public static CANSparkMax createSparkMaxFollower(CANSparkMax master, Configuration config) {
        CANSparkMax follower = new CANSparkMax(config.CANID, MotorType.kBrushless);
        if (config.voltageCompensation) {
            if (config.voltageCompensation) {
                handleCANError(config.CANID,
                        follower.enableVoltageCompensation(config.nominalVoltage),
                        "set voltage compensation");
            } else {
                handleCANError(config.CANID, follower.disableVoltageCompensation(),
                        "disable voltage compensation");
            }
        }
        handleCANError(config.CANID, follower.follow(master), "set follow");
        return follower;
    }

    public static CANSparkMax createSparkMax() {
        return createSparkMax(DEFAULT);
    }
}
