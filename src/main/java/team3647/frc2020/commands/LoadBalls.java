package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

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
        if(m_indexer.getBannerSensorValue()) {
            m_kickerWheel.setOpenloop(-0.15);
            m_indexer.set(IndexerSignal.GO_SLOW);
        } else {
            m_indexer.set(IndexerSignal.GO);
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
