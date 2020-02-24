/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package lib;

/**
 * Add your docs here.
 */
public class IndexerSignal {
    private final double funnelOutput;
    private final double tunnelOutput;
    private final double rollersOutput;

    public static IndexerSignal STOP = new IndexerSignal(0, 0, 0);
    public static IndexerSignal GO = new IndexerSignal(1, .7, .4);
    public static IndexerSignal GO_TUNNEL_STOP = new IndexerSignal(0, .7, 0);
    public static IndexerSignal SPITOUT = new IndexerSignal(-1, -1, -1);
    public static IndexerSignal TUNNELHOLD_GO = new IndexerSignal(.5, 0, 0);
    public static IndexerSignal GO_SLOW = new IndexerSignal(1, .58, .4);
    public static IndexerSignal GO_FAST = new IndexerSignal(1, 1, .6);
    public static IndexerSignal TUNNELDOWN_HOTDOGOUT = new IndexerSignal(-.7, -.7, -.5);
    public static IndexerSignal INDEXERFWD_SLOW = new IndexerSignal(1, .4, .5);

    public IndexerSignal(double funnelOutput, double tunnelOutput, double rollersOutput) {
        this.funnelOutput = funnelOutput;
        this.tunnelOutput = tunnelOutput;
        this.rollersOutput = rollersOutput;
    }

    public double getFunnelOutput() {
        return this.funnelOutput;
    }

    public double getTunnelOutput() {
        return this.tunnelOutput;
    }

    public double getRollersOutput() {
        return this.rollersOutput;
    }
}
