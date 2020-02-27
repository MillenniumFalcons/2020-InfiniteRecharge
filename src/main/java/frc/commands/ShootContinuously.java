/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.KickerWheel;
import lib.IndexerSignal;
import lib.wpi.Timer;

public class ShootContinuously extends CommandBase {

    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private final DoubleSupplier m_distanceToTarget;
    private final Timer voltageTooLowTimer = new Timer();
    private final BooleanSupplier shootAnyways;
    private double distanceToTargetMeters;
    private final Function<Double, Double> m_calculateRPMFromDistance;
    private boolean reachedVelOnce = false;
    private double motorOutputAtRPM = .7;

    /**
     * Shoot balls continuously while execute is ran
     */
    public ShootContinuously(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, BooleanSupplier shootAnyways,
            Function<Double, Double> calculateRPMFromDistance, DoubleSupplier distanceToTarget) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        m_distanceToTarget = distanceToTarget;
        m_calculateRPMFromDistance = calculateRPMFromDistance;
        this.shootAnyways = shootAnyways;

        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        voltageTooLowTimer.stop();
        voltageTooLowTimer.start();
        reachedVelOnce = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        distanceToTargetMeters = m_distanceToTarget.getAsDouble();

        // Far shot from behind cp:
        m_kickerWheel.setOpenloop(1);

        m_flywheel.setRPM(7300);

        if (m_flywheel.reachedTargetVelocity() || shootAnyways.getAsBoolean()) {
            if (m_indexer.getBannerSensorValue()) {
                m_indexer.set(IndexerSignal.GO_SLOW);
            } else {
                m_indexer.set(IndexerSignal.GO);
            }
            voltageTooLowTimer.reset();
        } else {
            if (!voltageTooLowTimer.isRunning()) {
                voltageTooLowTimer.start();
            }
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
        m_flywheel.end();
        m_kickerWheel.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
