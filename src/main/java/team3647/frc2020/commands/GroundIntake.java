package team3647.frc2020.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Intake;

public class GroundIntake extends CommandBase {
    private final Intake m_intake;
    private final DoubleSupplier drivetrainDemand;

    /**
     * Creates a new GroundIntake.
     */
    public GroundIntake(Intake intake, DoubleSupplier drivetrainDemand) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_intake = intake;
        this.drivetrainDemand = drivetrainDemand;
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
        m_intake.extendOuter();
        m_intake.extendInner();
        m_intake.intake(.7 + Math.abs(drivetrainDemand.getAsDouble()) * .7);
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
