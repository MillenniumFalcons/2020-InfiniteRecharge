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

/**
 * Add your docs here.
 */
public class Indexer implements PeriodicSubsystem {

    private VictorSPX funnel;
    private VictorSPX tunnel;
    private VictorSPX rollers;

    public Indexer() {
    }


    public void set(IndexerSignal signal) {
        funnel.set(ControlMode.PercentOutput, signal.getFunnelOutput());
        tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput());
        rollers.set(ControlMode.PercentOutput, signal.getrollerOutput());
    }

    @Override
    public String getName() {
        return "Indexer";
    }
}
