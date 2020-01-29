/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import lib.IndexerSignal;
import lib.drivers.VictorSPXFactory;
import lib.wpi.HALMethods;

/**
 * Add your docs here.
 */
public class Indexer implements PeriodicSubsystem {

    private final VictorSPX funnel;
    private final VictorSPX tunnel;
    private final VictorSPX rollers;

    public Indexer(VictorSPXFactory.Configuration funnelConfig, VictorSPXFactory.Configuration tunnelConfig,
            VictorSPXFactory.Configuration rollersConfig) {
        boolean error = false;
        if (funnelConfig == null) {
            HALMethods.sendDSError("funnel config was null");
            error = true;
        }
        if (tunnelConfig == null) {
            HALMethods.sendDSError("tunnel config was null");
            error = true;
        }
        if (rollersConfig == null) {
            HALMethods.sendDSError("rollers config was null");
            error = true;
        }

        if (error) {
            throw new IllegalArgumentException("1 or more of the arguments to Indexer constructor were null");
        } else {
            funnel = VictorSPXFactory.createVictor(funnelConfig);
            tunnel = VictorSPXFactory.createVictor(tunnelConfig);
            rollers = VictorSPXFactory.createVictor(rollersConfig);
        }
    }

    public void set(IndexerSignal signal) {
        if (signal == null) {
            HALMethods.sendDSError("Indexer signal was null!");
            return;
        }
        funnel.set(ControlMode.PercentOutput, signal.getFunnelOutput());
        tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput());
        rollers.set(ControlMode.PercentOutput, signal.getrollerOutput());
    }

    @Override
    public String getName() {
        return "Indexer";
    }
}
