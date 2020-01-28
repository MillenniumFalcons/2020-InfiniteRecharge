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
        public final boolean inverted;

        public boolean enableCurrentLimiting = false;
        public int peakCurrent = 0;
        public int peakCurrentDuration = 0;
        public int continuousCurrent = 0;
        public NeutralMode neutralMode = NeutralMode.Coast;
        public boolean voltageCompensation = false;
        public double nominalVoltage = 0.0;

        private Configuration(int CANID, boolean inverted) {
            this.CANID = CANID;
            this.inverted = inverted;
        }

        public Configuration currentLimiting(int peakCurrent, int peakCurrentDuration,
                int continuousCurrent) {
            enableCurrentLimiting = true;
            this.peakCurrent = peakCurrent;
            this.peakCurrentDuration = peakCurrentDuration;
            this.continuousCurrent = continuousCurrent;
            return this;
        }

        public Configuration voltageCompensation(double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            this.voltageCompensation = true;
            return this;
        }

        public Configuration neutralMode(NeutralMode mode) {
            this.neutralMode = mode;
            return this;
        }
    }

    private static final Configuration DEFAULT = new Configuration(0, false);


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
