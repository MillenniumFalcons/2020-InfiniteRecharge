/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Commands.AimTurret;
import frc.Commands.ArcadeDrive;
import frc.Commands.GroundIntake;
import frc.Commands.LoadBalls;
import frc.Commands.LoadingStationIntake;
import frc.Commands.OrganizeFeeder;
import frc.Commands.ShootContinuously;
import frc.Commands.StowIntakeAndOrganizeFeeder;
import frc.Commands.TurretMotionMagic;
import frc.team3647Subsystems.Drivetrain;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.Intake;
import frc.team3647Subsystems.KickerWheel;
import frc.team3647Subsystems.Turret;
import frc.team3647Subsystems.VisionController;
import frc.team3647inputs.Joysticks;
import lib.IndexerSignal;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Joysticks mainController = new Joysticks(0);
    private Joysticks coController = new Joysticks(1);

    // private final VisionController m_visionController =
    // new VisionController("10.36.47.86", Constants.cVisionController.camConstants);

    // private final Drivetrain m_drivetrain = new
    // Drivetrain(Constants.cDrivetrain.leftMasterConfig,
    // Constants.cDrivetrain.rightMasterConfig, Constants.cDrivetrain.leftSlaveConfig,
    // Constants.cDrivetrain.rightSlaveConfig, Constants.cDrivetrain.leftMasterPIDConfig,
    // Constants.cDrivetrain.rightMasterPIDConfig, Constants.cDrivetrain.shifterPin,
    // Constants.cDrivetrain.kWheelDiameter);

    private final Flywheel m_flywheel = new Flywheel(Constants.cFlywheel.masterConfig,
            Constants.cFlywheel.slaveConfig, Constants.cFlywheel.pidConfig);

    private final Indexer m_indexer =
            new Indexer(Constants.cIndexer.funnelConfig, Constants.cIndexer.tunnelConfig,
                    Constants.cIndexer.rollersConfig, Constants.cIndexer.bannerSensorPin);

    private final Intake m_intake = new Intake(Constants.cIntake.intakeMotorConfig,
            Constants.cIntake.innerPistonsPin, Constants.cIntake.outerPistonsPin);

    private final KickerWheel m_kickerWheel =
            new KickerWheel(Constants.cKickerWheel.masterConfig, Constants.cKickerWheel.pidConfig);

    // private final Turret m_turret = new Turret(Constants.cTurret.masterConfig,
    // Constants.cTurret.pidConfig, Constants.cTurret.kMaxRotationDeg,
    // Constants.cTurret.kMinRotationDeg, Constants.cTurret.limitSwitchPin);

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
                            Constants.cDrivetrain.kA),
                    Constants.cDrivetrain.kDriveKinematics, Constants.cDrivetrain.maxVoltage);

    // Create config for trajectory
    private final TrajectoryConfig trajectoryConfig =
            new TrajectoryConfig(Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
                    Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                            // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                            // Apply the voltage constraint
                            .addConstraint(autoVoltageConstraint);

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    public RobotContainer() {
        // m_commandScheduler.registerSubsystem(m_visionController);
        // m_commandScheduler.registerSubsystem(m_drivetrain);

        m_commandScheduler.registerSubsystem(m_flywheel);
        m_commandScheduler.registerSubsystem(m_indexer);
        m_commandScheduler.registerSubsystem(m_intake);
        m_commandScheduler.registerSubsystem(m_kickerWheel);
        SmartDashboard.putNumber("Shooter rpm", m_flywheel.getVelocity());
        SmartDashboard.putNumber("Shooter demand", 0);

        m_commandScheduler.setDefaultCommand(m_flywheel, new RunCommand(() -> {
            m_flywheel.setOpenloop(mainController.getLeftStickY() * .9);
            SmartDashboard.putNumber("Shooter rpm", m_flywheel.getVelocity());
            SmartDashboard.putNumber("Shooter rpm", mainController.getLeftStickY() * .9);
        }, m_flywheel));


        m_commandScheduler.setDefaultCommand(m_indexer, new RunCommand(() -> {
            m_indexer.set(new IndexerSignal(mainController.getRightStickY(), mainController.getRightStickY() * .5, -mainController.getRightStickY() * .5));
                    // mainController.getRightStickY(), mainController.getRightStickY()));
        }, m_indexer));

        m_commandScheduler.setDefaultCommand(m_intake, new RunCommand(() -> {
            m_intake.intake(mainController.rightTrigger.getTriggerValue());
        }, m_intake));

        m_commandScheduler.setDefaultCommand(m_kickerWheel, new RunCommand(() -> {
            m_kickerWheel.setOpenloop(mainController.getLeftStickY() );
        }, m_kickerWheel));
        // m_commandScheduler.setDefaultCommand(m_drivetrain,
        // new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
        // mainController::getRightStickX, mainController.rightJoyStickPress::get));
        // configButtonBindings();
    }

    private void configButtonBindings() {
        // coController.leftTrigger.whenActive(new ParallelCommandGroup(
        // new GroundIntake(m_intake, coController.leftTrigger::getTriggerValue),
        // new LoadBalls(m_kickerWheel, m_indexer)));

        // coController.leftBumper.whenActive(new ParallelCommandGroup(
        // new LoadingStationIntake(m_intake), new LoadBalls(m_kickerWheel, m_indexer)));

        // coController.leftTrigger
        // .whenReleased(new StowIntakeAndOrganizeFeeder(m_intake, m_indexer, m_kickerWheel));
        // coController.leftBumper
        // .whenReleased(new OrganizeFeeder(m_indexer, m_kickerWheel).withTimeout(3));

        // coController.rightBumper.whenActive(new AimTurret(m_turret, m_visionController::getYaw));
        // coController.rightTrigger.whenActive(new ShootContinuously(m_flywheel, m_kickerWheel,
        // m_indexer, Constants.cFlywheel::calculateRPM, m_visionController::getDistance));

        // coController.dPadLeft
        // .whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.leftDeg));
        // coController.dPadRight
        // .whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.rightDeg));
        // coController.dPadUp
        // .whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.forwardDeg));
        // coController.dPadDown
        // .whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.backwardDeg));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
