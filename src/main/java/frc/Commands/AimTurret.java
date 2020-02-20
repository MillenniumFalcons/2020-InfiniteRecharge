/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Turret;

public class AimTurret extends CommandBase {

    private final Turret m_turret;
    private final DoubleSupplier m_angleToTarget;

    /**
     * Creates a new AimTurret.
     */
    public AimTurret(Turret turret, DoubleSupplier angleToTurret) {
        m_turret = turret;
        m_angleToTarget = angleToTurret;
        addRequirements(m_turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_turret.setAngle(m_turret.getAngle() - m_angleToTarget.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
