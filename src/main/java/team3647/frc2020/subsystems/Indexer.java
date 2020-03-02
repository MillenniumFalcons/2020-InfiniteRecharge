/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import java.util.function.Function;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.IndexerSignal;
import team3647.lib.drivers.VictorSPXFactory;
import team3647.lib.wpi.HALMethods;

/**
 * feeder boi.
 */
public class Indexer implements PeriodicSubsystem {

    private final VictorSPX PP_Vertical;
    private final VictorSPX tunnel;
    private final VictorSPX rollers_vertical;

    private final int funnelPDPSlot;
    private final int tunnelPDPSlot;
    private final int rollersPDPSlot;

    private final DigitalInput bannerSensor;
    private boolean bannerSensorValue;

    private final Function<Integer, Double> getCurrent;

    public Indexer(VictorSPXFactory.Configuration funnelConfig, VictorSPXFactory.Configuration tunnelConfig,
            VictorSPXFactory.Configuration rollersConfig, int bannerSensorPin, Function<Integer, Double> getCurrent) {
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
        if (getCurrent == null) {
            HALMethods.sendDSError(getName() + "getcurrent was null");
            error = true;
        }

        if (error) {
            throw new NullPointerException("1 or more of the arguments to Indexer constructor were null");
        } else {
            PP_Vertical = VictorSPXFactory.createVictor(funnelConfig);
            tunnel = VictorSPXFactory.createVictor(tunnelConfig);
            rollers_vertical = VictorSPXFactory.createVictor(rollersConfig);
            this.getCurrent = getCurrent;
        }

        funnelPDPSlot = funnelConfig.pdpSlot;
        tunnelPDPSlot = tunnelConfig.pdpSlot;
        rollersPDPSlot = rollersConfig.pdpSlot;

        bannerSensor = new DigitalInput(bannerSensorPin);
    }

    /**
     * @return the rollersPDPSlot
     */
    public int getRollersPDPSlot() {
        return rollersPDPSlot;
    }

    /**
     * @return the tunnelPDPSlot
     */
    public int getTunnelPDPSlot() {
        return tunnelPDPSlot;
    }

    /**
     * @return the funnelPDPSlot
     */
    public int getFunnelPDPSlot() {
        return funnelPDPSlot;
    }

    public void set(IndexerSignal signal) {
        if (signal == null) {
            HALMethods.sendDSError("Indexer signal was null!");
            return;
        }
        if (getCurrent.apply(funnelPDPSlot) > 30) {
            PP_Vertical.set(ControlMode.PercentOutput, signal.getPP_VerticalOutput() * .5);
        } else {
            PP_Vertical.set(ControlMode.PercentOutput, signal.getPP_VerticalOutput());
        }
        if (getCurrent.apply(tunnelPDPSlot) > 30) {
            tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput() * .5);
        } else {
            tunnel.set(ControlMode.PercentOutput, signal.getTunnelOutput());
        }
        rollers_vertical.set(ControlMode.PercentOutput, signal.getRollers_verticalOutput());
    }

    @Override
    public void periodic() {
        bannerSensorValue = !bannerSensor.get();
    }

    public boolean getBannerSensorValue() {
        return bannerSensorValue;
    }

    @Override
    public void end() {
        set(IndexerSignal.STOP);
    }

    @Override
    public String getName() {
        return "Indexer";
    }
}
