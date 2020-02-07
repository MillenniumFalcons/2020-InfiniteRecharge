/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.KickerWheel;
import lib.IndexerSignal;

public class LoadBalls extends CommandBase {
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;

    /**
     * Creates a new LoadBalls.
     */
    public LoadBalls(KickerWheel kickerWheel, Indexer indexer) {
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        addRequirements(m_kickerWheel, m_indexer);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_indexer.set(IndexerSignal.GO);

        if(m_indexer.getBannerSensorValue()) {
            m_kickerWheel.setOpenloop(-0.3);
        } else {
            m_kickerWheel.end();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_indexer.end();
        m_kickerWheel.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
