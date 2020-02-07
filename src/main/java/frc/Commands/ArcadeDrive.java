/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Drivetrain;
import lib.wpi.HALMethods;

public class ArcadeDrive extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_throttle;
  private final DoubleSupplier m_turn;
  private final BooleanSupplier m_scaleInputs;

  /**
   * Creates a new ArcadeDrive.
   */
  public ArcadeDrive(Drivetrain drivetrain, DoubleSupplier throttle, DoubleSupplier turn,
      BooleanSupplier scaleInputs) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
    boolean error = false;
    if (throttle == null) {
      HALMethods.sendDSError("throttle was null");
      error = true;
    }

    if (turn == null) {
      HALMethods.sendDSError("turn was null");
      error = true;
    }

    if (scaleInputs == null) {
      HALMethods.sendDSError("Scale inputs was null");
      error = true;
    }

    if (error) {
      throw new NullPointerException("one or more of the arguments were null");
    }

    m_throttle = throttle;
    m_turn = turn;
    m_scaleInputs = scaleInputs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_throttle.getAsDouble(), m_turn.getAsDouble(),
        m_scaleInputs.getAsBoolean());
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
