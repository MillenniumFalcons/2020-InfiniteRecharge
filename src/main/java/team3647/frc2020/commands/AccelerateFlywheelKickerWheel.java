/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2020.subsystems.Flywheel;

public class AccelerateFlywheelKickerWheel extends CommandBase {
    private final Flywheel m_flywheel;
    private final DoubleSupplier flywheelRPM;
    /**
     * Creates a new AccelerateFlywheel.
     */
    public AccelerateFlywheelKickerWheel(Flywheel flywheel,
            DoubleSupplier flywheelRPM) {
        m_flywheel = flywheel;
        this.flywheelRPM = flywheelRPM;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_flywheel.setRPM(flywheelRPM.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_flywheel.reachedTargetVelocity();
    }
}
