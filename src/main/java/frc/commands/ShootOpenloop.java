/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.KickerWheel;
import lib.IndexerSignal;
import lib.team3647Utils.RollingAverage;

public class ShootOpenloop extends CommandBase {
    private final Flywheel m_flywheel;
    private final KickerWheel m_kickerWheel;
    private final Indexer m_indexer;
    private boolean reachedVelOnce;
    private double motorOutputAtRPM = 1;
    private final DoubleSupplier rpmSupplier;
    private final DoubleSupplier kickerWheelOutputSupplier;
    private double lastVelRequested;
    private double velRequested;

    // private final Roll
    /**
     * Shoot balls continuously while execute is ran
     */
    public ShootOpenloop(Flywheel flywheel, KickerWheel kickerWheel, Indexer indexer, DoubleSupplier rpmSupplier,
            DoubleSupplier kickerWheelOutputSupplier) {
        m_flywheel = flywheel;
        m_kickerWheel = kickerWheel;
        m_indexer = indexer;
        this.rpmSupplier = rpmSupplier;
        this.kickerWheelOutputSupplier = kickerWheelOutputSupplier;
        addRequirements(m_flywheel, m_kickerWheel, m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        reachedVelOnce = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        lastVelRequested = velRequested;
        velRequested = rpmSupplier.getAsDouble();

        if (Math.abs(lastVelRequested - velRequested) > 50) {
            reachedVelOnce = false;
        }

        // AUTO SHOT
        m_kickerWheel.setOpenloop(kickerWheelOutputSupplier.getAsDouble());

        if (m_flywheel.getVelocity() > velRequested) {
            motorOutputAtRPM = m_flywheel.getOutput();
            reachedVelOnce = true;

        } else if (!reachedVelOnce) {
            m_flywheel.setRPM(velRequested);
        }

        if (reachedVelOnce) {
            m_indexer.set(IndexerSignal.GO_FAST);
            m_flywheel.setOpenloop(motorOutputAtRPM * 1.02);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_flywheel.end();
        m_kickerWheel.end();
        m_indexer.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // AUTO SHOT
    // m_kickerWheel.setOpenloop(.35);

    // if (m_flywheel.getVelocity() > 5500) {
    // motorOutputAtRPM = m_flywheel.getOutput();
    // reachedVelOnce = true;

    // } else if (!reachedVelOnce) {
    // m_flywheel.setRPM(5500);
    // }

    // if (reachedVelOnce) {
    // m_indexer.set(IndexerSignal.GO_FAST);
    // m_flywheel.setOpenloop(motorOutputAtRPM * 1.02);
    // }
    // ------------------------------------------------------------------
    // initiation line : (2 rollers #4) battery 12.6 disabled

    // m_kickerWheel.setOpenloop(.35);

    // if (m_flywheel.getVelocity() > 4300) {
    // motorOutputAtRPM = m_flywheel.getOutput();
    // reachedVelOnce = true;

    // } else if (!reachedVelOnce) {
    // m_flywheel.setRPM(4300);
    // }

    // if (reachedVelOnce) {
    // m_indexer.set(IndexerSignal.GO_FAST);
    // m_flywheel.setOpenloop(motorOutputAtRPM * 1.03);
    // }
    // ------------------------------------------------------------------

}
