/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import lib.wpi.HALMethods;

/**
 * Create victor with configs.
 */
public class VictorSPXFactory {

    public static class Configuration {
        public final int CANID;
        public boolean inverted;
        public double maxOutput;
        public double minOutput;

        public Configuration(int CANID) {
            this.CANID = CANID;
            inverted = false;
            maxOutput = 1;
        }

        public Configuration setInverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }


        /**
         * @param maxOuput is [0, 1]
         */
        public Configuration configMaxOutput(double maxOutput) {
            if (maxOutput < 1 && maxOutput > 0) {
                this.maxOutput = maxOutput;
            }
            return this;
        }

        /**
         * @param minOutput is [-1, 0]
         */
        public Configuration configMaxReverseOutput(double minOutput) {
            if (minOutput < 0 && minOutput > -1) {
                this.minOutput = minOutput;
            }
            return this;
        }
    }

    private static void handleCANError(int id, ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            HALMethods.sendDSError("Could not configure talon id: " + id + " error: "
                    + error.toString() + " " + message);
        }
    }

    public static VictorSPX createVictor(Configuration config) {
        VictorSPX victor = new VictorSPX(config.CANID);
        victor.configFactoryDefault();
        victor.setInverted(config.inverted);
        System.out.println("created victor id: " + config.CANID);

        handleCANError(config.CANID, victor.configPeakOutputForward(config.maxOutput),
                "set max forward output");
        handleCANError(config.CANID, victor.configPeakOutputReverse(config.minOutput),
                "set max reverse output");
        return victor;
    }
}
