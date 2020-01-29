/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package lib.drivers;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Create victor with configs.
 */
public class VictorSPXFactory {

    public static class Configuration {
        public int CANID;
        public boolean inverted;

        public Configuration(int CANID) {
            this.CANID = CANID;
        }

        public Configuration setInverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }
    }

    public static VictorSPX createVictor(Configuration config) {
        VictorSPX victor = new VictorSPX(config.CANID);
        victor.setInverted(config.inverted);
        return victor;
    }
}
