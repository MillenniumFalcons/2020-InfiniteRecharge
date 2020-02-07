/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import lib.drivers.ClosedLoopFactory;
import lib.drivers.TalonSRXFactory;

/**
 * spinny up.
 */
public class KickerWheel extends TalonSRXSubsystem {

    public KickerWheel(TalonSRXFactory.Configuration masterConfig,
            ClosedLoopFactory.ClosedLoopConfig pidConfig) {
        super(masterConfig, pidConfig);
    }

    public void setRPM(double RPM) {
        setVelocity(RPM);
    }

    @Override
    public String getName() {
        return "KickerWheel";
    }


}
