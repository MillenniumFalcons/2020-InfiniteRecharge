/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Intake;

public class GroundIntake extends CommandBase {
    private final Intake m_intake;
    private final DoubleSupplier m_demand;

    /**
     * Creates a new GroundIntake.
     */
    public GroundIntake(Intake intake, DoubleSupplier demand) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_demand = demand;
        m_intake = intake;
        addRequirements(m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_intake.extendOuter();
        m_intake.extendInner();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_intake.intake(m_demand.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_intake.retractInner();
        // m_intake.retractOuter();
        // m_intake.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
