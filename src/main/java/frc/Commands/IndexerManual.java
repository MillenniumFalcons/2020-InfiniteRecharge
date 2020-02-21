/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Indexer;
import lib.IndexerSignal;

public class IndexerManual extends CommandBase {
    private final DoubleSupplier outputSupplier;
    private final Indexer m_indexer;

    /**
     * Creates a new IndexerManual.
     */
    public IndexerManual(Indexer indexer, DoubleSupplier output) {
        m_indexer = indexer;
        this.outputSupplier = output;
        addRequirements(m_indexer);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double output = outputSupplier.getAsDouble();
        m_indexer.set(new IndexerSignal(Math.signum(output), output, output));
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
