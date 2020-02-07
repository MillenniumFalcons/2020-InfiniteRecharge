/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.KickerWheel;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class OrganizeFeeder extends SequentialCommandGroup {
    /**
     * Move the balls back until the banner sensor is clear then moves forward until is true, to
     * reorgranize before shooting so we don't shoot a ball while the kicker wheel is accelerating
     */
    public OrganizeFeeder(Indexer indexer, KickerWheel kickerWheel) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());
        super(new RollTunnelBack(indexer, kickerWheel), new RollTunnelForwards(indexer).withTimeout(.5));
    }
}
