package lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application. (254)
 */
public class TalonSRXFactory {

    private final static int kTimeoutMs = 100;


    public static class Configuration {
        public final int CANID;
        public final int peakCurrent;
        public final int peakCurrentDuration;
        public final int continuousCurrent;
        public final boolean inverted;
        public final NeutralMode neutralMode;
        public final boolean voltageCompensation;
        public final double nominalVoltage;
        public final boolean enableCurrentLimiting;

        private Configuration() {
            this.CANID = 0;
            this.peakCurrent = 1;
            this.peakCurrentDuration = 0;
            this.continuousCurrent = 1;
            this.inverted = false;
            this.neutralMode = NeutralMode.Coast;
            this.voltageCompensation = false;
            this.nominalVoltage = 12.0;
            this.enableCurrentLimiting = false;
        }

        public Configuration(int CANID, int peakCurrent, int peakCurrentDuration,
                int continuousCurrent, boolean inverted, NeutralMode neutralMode,
                boolean voltageCompensation, double nominalVoltage, boolean enableCurrentLimiting) {
            this.CANID = CANID;
            this.peakCurrent = peakCurrent;
            this.peakCurrentDuration = peakCurrentDuration;
            this.continuousCurrent = continuousCurrent;
            this.inverted = inverted;
            this.neutralMode = neutralMode;
            this.voltageCompensation = voltageCompensation;
            this.nominalVoltage = nominalVoltage;
            this.enableCurrentLimiting = enableCurrentLimiting;
        }
    }

    private static final Configuration DEFAULT = new Configuration();


    private static void handleCANError(int id, ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError("Could not configure talon id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonSRX createDefaultTalon() {
        return createTalon(DEFAULT);
    }

    public static TalonSRX createTalon(Configuration config) {
        TalonSRX talon = new TalonSRX(config.CANID);
        talon.set(ControlMode.PercentOutput, 0.0);
        handleCANError(config.CANID, talon.configFactoryDefault(), "restore factory defaults");
        talon.clearStickyFaults(kTimeoutMs);

        talon.enableCurrentLimit(config.enableCurrentLimiting);
        handleCANError(config.CANID, talon.configPeakCurrentLimit(config.peakCurrent),
                "set peak current");
        handleCANError(config.CANID, talon.configPeakCurrentDuration(config.peakCurrentDuration),
                "set peak current duration");
        handleCANError(config.CANID, talon.configContinuousCurrentLimit(config.continuousCurrent),
                "set continuous current");

        handleCANError(config.CANID, talon.configVoltageCompSaturation(config.nominalVoltage),
                "set nominal voltage");
        talon.enableVoltageCompensation(config.voltageCompensation);
        return talon;
    }
}
