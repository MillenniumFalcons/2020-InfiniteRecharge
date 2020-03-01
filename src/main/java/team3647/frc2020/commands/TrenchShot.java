/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

// .65 position on servo 22ft 1"
public class TrenchShot extends ShootClosedLoop {
    /**
     * Creates a new TrenchShot.
     */
    public TrenchShot(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer) {
        super(flywheel, kickerWheel, indexer, 6300, .65, IndexerSignal.GO_SLOW);
    }
}
