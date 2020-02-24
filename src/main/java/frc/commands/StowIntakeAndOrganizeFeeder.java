/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.Intake;
import frc.team3647Subsystems.KickerWheel;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StowIntakeAndOrganizeFeeder extends SequentialCommandGroup {
    /**
     * Creates a new StowIntakeAndOrganizeFeeder.
     */
    public StowIntakeAndOrganizeFeeder(Intake intake, Indexer indexer, KickerWheel kickerWheel) {
        super(new StowOuterPistons(intake).withTimeout(.3), new RunIntakeRoller(intake, .3).withTimeout(.4),
                new WaitCommand(2), new OrganizeFeeder(indexer, kickerWheel).withTimeout(3));
    }
}
