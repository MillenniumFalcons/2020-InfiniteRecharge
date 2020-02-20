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
    public static IndexerSignal INDEXERBACK = new IndexerSignal(.7, -.7, -.3);
    public static IndexerSignal INDEXERFORWARDS = new IndexerSignal(1, .5, .5);
    public static IndexerSignal SPITOUT = new IndexerSignal(-1, -1, -1);
    public static IndexerSignal TUNNELBACK_HOTDOGFWD= new IndexerSignal(.7, -.7, .7);
    public static IndexerSignal TUNNELHOLD_GO = new IndexerSignal(.7, 0, .5);
    public static IndexerSignal GO_SLOW = new IndexerSignal(1, .4, .4);

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
