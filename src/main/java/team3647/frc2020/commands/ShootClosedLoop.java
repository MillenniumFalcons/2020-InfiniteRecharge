/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.lib.IndexerSignal;

public class ShootClosedLoop extends CommandBase {
    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final double shooterRPM;
    private final double kickerWheelOutput;
    private final IndexerSignal signalOnShoot;

    /**
     * Creates a new ShootClosedLoop.
     */
    public ShootClosedLoop(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer,
            double shooterRPM, double kickerWheelOutput, IndexerSignal signalOnShoot) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;

        this.kickerWheelOutput = kickerWheelOutput;
        this.shooterRPM = shooterRPM;
        this.signalOnShoot = signalOnShoot;
        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_kickerWheel.setOpenloop(kickerWheelOutput);

        m_flywheel.setRPM(shooterRPM);

        if (m_flywheel.reachedTargetVelocity()) {
            if (m_indexer.getBannerSensorValue()) {
                m_indexer.set(signalOnShoot);
            } else {
                m_indexer.set(signalOnShoot);
            }
        } else {
            if (m_indexer.getBannerSensorValue()) {
                m_indexer.set(IndexerSignal.TUNNELHOLD_GO);
            } else {
                m_indexer.set(IndexerSignal.GO_SLOW);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
