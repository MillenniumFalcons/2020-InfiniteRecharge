/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Intake;

public class ExtendIntakeToGround extends CommandBase {

    private final Intake m_intake;
  /**
   * Creates a new ExtendIntakeToGround.
   */
  public ExtendIntakeToGround(Intake intake) {
      m_intake = intake;
      addRequirements(m_intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.extendInner();
    m_intake.extendOuter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_intake.extendInner();
      m_intake.extendOuter();
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