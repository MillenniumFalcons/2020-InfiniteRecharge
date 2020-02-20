/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.Intake;
import frc.team3647Subsystems.KickerWheel;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StowIntakeAndOrganizeFeeder extends ParallelCommandGroup {
    /**
     * Creates a new StowIntakeAndOrganizeFeeder.
     */
    public StowIntakeAndOrganizeFeeder(Intake intake, Indexer indexer, KickerWheel kickerWheel) {
        // Add your commands in the super() call, e.g.
        // super(new FooCommand(), new BarCommand());super();
        super(new SequentialCommandGroup(new StowOuterPistons(intake).withTimeout(.3),
                new RunIntakeRoller(intake, .3).withTimeout(.5)),
                new OrganizeFeeder(indexer, kickerWheel).withTimeout(3));
    }
}
