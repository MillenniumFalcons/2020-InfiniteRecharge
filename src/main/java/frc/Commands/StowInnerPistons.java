/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3647Subsystems.Intake;

public class StowInnerPistons extends CommandBase {
    private final Intake m_intake;
  /**
   * Creates a new StowInnerPistons.
   */
  public StowInnerPistons(Intake intake) {
      m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_intake.retractInner();
  }
}
