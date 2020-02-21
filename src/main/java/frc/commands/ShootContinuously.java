/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.KickerWheel;
import lib.IndexerSignal;

public class ShootContinuously extends CommandBase {

    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final DoubleSupplier m_distanceToTarget;
    private double distanceToTargetMeters;
    private final Function<Double, Double> m_calculateRPMFromDistance;

    /**
     * Shoot balls continuously while execute is ran
     */
    public ShootContinuously(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer,
            Function<Double, Double> calculateRPMFromDistance, DoubleSupplier distanceToTarget) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        m_distanceToTarget = distanceToTarget;
        m_calculateRPMFromDistance = calculateRPMFromDistance;

        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        distanceToTargetMeters = m_distanceToTarget.getAsDouble();

        m_flywheel.setRPM(7700);
        //7270rpm 2 blue fairlanes 1.5" foam .6 kicker wheel 12.5v battery
        // m_kickerWheel.setRPM(m_calculateRPMFromDistance.apply(distanceToTargetMeters));
        m_kickerWheel.setOpenloop(.6);

        if(m_flywheel.reachedTargetVelocity()) {
            // if(m_indexer.getBannerSensorValue()) {
            //     m_indexer.set(IndexerSignal.GO_SLOW);
            // } else {
            //     m_indexer.set(IndexerSignal.GO);
            // }
            m_indexer.set(IndexerSignal.GO);
        } else {
            if(m_indexer.getBannerSensorValue()) {
                m_indexer.set(IndexerSignal.TUNNELHOLD_GO);
            } else {
                m_indexer.set(IndexerSignal.GO_SLOW);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_flywheel.end();
        m_kickerWheel.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
