/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AimTurret;
import frc.commands.ArcadeDrive;
import frc.commands.ExtendIntakeToGround;
import frc.commands.GroundIntake;
import frc.commands.IndexerManual;
import frc.commands.LoadBalls;
import frc.commands.LoadingStationIntake;
import frc.commands.OrganizeFeeder;
import frc.commands.RemoveBalls;
import frc.commands.ShootContinuously;
import frc.commands.StowIntakeAndOrganizeFeeder;
import frc.commands.TurretManual;
import frc.commands.TurretMotionMagic;
import frc.team3647Subsystems.Drivetrain;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.Intake;
import frc.team3647Subsystems.KickerWheel;
import frc.team3647Subsystems.Turret;
import frc.team3647Subsystems.VisionController;
import frc.team3647inputs.Joysticks;
import lib.GroupPrinter;
import lib.wpi.Compressor;
import lib.wpi.PDP;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private Joysticks mainController = new Joysticks(0);
    private Joysticks coController = new Joysticks(1);
    private final PDP pdp = new PDP();
    private Compressor airCompressor = new Compressor(0);

    private final VisionController m_visionController = new VisionController(Constants.cVisionController.camIP,
            Constants.cVisionController.camConstants);

    private final Drivetrain m_drivetrain = new Drivetrain(Constants.cDrivetrain.leftMasterConfig,
            Constants.cDrivetrain.rightMasterConfig, Constants.cDrivetrain.leftSlaveConfig,
            Constants.cDrivetrain.rightSlaveConfig, Constants.cDrivetrain.leftMasterPIDConfig,
            Constants.cDrivetrain.rightMasterPIDConfig, Constants.cDrivetrain.shifterPin,
            Constants.cDrivetrain.kWheelDiameter, Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
            Constants.cDrivetrain.kA, 16);

    private final Flywheel m_flywheel = new Flywheel(Constants.cFlywheel.masterConfig, Constants.cFlywheel.slaveConfig,
            Constants.cFlywheel.pidConfig);

    private final Indexer m_indexer = new Indexer(Constants.cIndexer.funnelConfig, Constants.cIndexer.tunnelConfig,
            Constants.cIndexer.rollersConfig, Constants.cIndexer.bannerSensorPin, pdp::getCurrent);

    private final Intake m_intake = new Intake(Constants.cIntake.intakeMotorConfig, Constants.cIntake.innerPistonsPin,
            Constants.cIntake.outerPistonsPin);

    private final KickerWheel m_kickerWheel = new KickerWheel(Constants.cKickerWheel.masterConfig,
            Constants.cKickerWheel.pidConfig);

    private final Turret m_turret = new Turret(Constants.cTurret.masterConfig, Constants.cTurret.pidConfig,
            Constants.cTurret.kMaxRotationDeg, Constants.cTurret.kMinRotationDeg, Constants.cTurret.limitSwitchPin);

    private final GroupPrinter m_printer = GroupPrinter.getInstance();

    private final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV, Constants.cDrivetrain.kA),
            Constants.cDrivetrain.kDriveKinematics, Constants.cDrivetrain.maxVoltage);

    // Create config for trajectory
    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.cDrivetrain.kMaxSpeedMetersPerSecond,
            Constants.cDrivetrain.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.cDrivetrain.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint).setReversed(true);

    // An example trajectory to follow. All units in meters.
    Trajectory sectorLineTo2Balls = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),

            List.of(),

            new Pose2d(-3.048, 0, new Rotation2d(0)),
            // Pass config
            trajectoryConfig);

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    public RobotContainer() {
        // setTestingIndexer();
        airCompressor.start();
        pdp.clearStickyFaults();
        // m_intake.extendInner();
        m_commandScheduler.registerSubsystem(m_kickerWheel, m_flywheel, m_visionController, m_intake, m_indexer,
                m_drivetrain, m_printer);
        m_commandScheduler.setDefaultCommand(m_drivetrain, new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
                mainController::getRightStickX, mainController.rightJoyStickPress::get));
        m_commandScheduler.setDefaultCommand(m_turret, new TurretManual(m_turret, coController::getLeftStickX));
        m_indexer.setDefaultCommand(new IndexerManual(m_indexer, coController::getRightStickY));
        // m_printer.addDouble("shooter rpm", m_flywheel::getVelocity);
        m_printer.addDouble("tunnel amps", () -> {
            return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
        });
        m_printer.addDouble("funnel amps", () -> {
            return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
        });
        m_printer.addDouble("kicker wheel amps", m_kickerWheel::getMasterCurrnet);
        configButtonBindings();
    }

    private void configButtonBindings() {
        coController.leftTrigger.whenActive(new SequentialCommandGroup(new RunCommand(() -> {
            m_intake.retractInner();
        }, m_intake).withTimeout(.5), new ExtendIntakeToGround(m_intake).withTimeout(.25), new ParallelCommandGroup(
                new GroundIntake(m_intake, mainController::getLeftStickY), new LoadBalls(m_kickerWheel, m_indexer))));

        coController.leftTrigger.whenReleased(new StowIntakeAndOrganizeFeeder(m_intake, m_indexer, m_kickerWheel));

        coController.leftBumper.whenActive(
                new ParallelCommandGroup(new LoadingStationIntake(m_intake), new LoadBalls(m_kickerWheel, m_indexer)));
        coController.leftBumper.whenReleased(new OrganizeFeeder(m_indexer, m_kickerWheel).withTimeout(3));

        coController.buttonA.whenActive(new RemoveBalls(m_indexer, m_intake, m_kickerWheel));
        coController.buttonA.whenReleased(new RunCommand(() -> {
            m_intake.end();
            m_indexer.end();
            m_kickerWheel.end();
            m_intake.retractOuter();
            m_intake.extendInner();
        }, m_indexer, m_intake, m_kickerWheel).withTimeout(.5));

        coController.rightBumper.whenActive(new AimTurret(m_turret, m_visionController::getFilteredYaw));
        coController.rightTrigger
                .whenActive(new ParallelCommandGroup(new AimTurret(m_turret, m_visionController::getFilteredYaw),
                        new ShootContinuously(m_flywheel, m_kickerWheel, m_indexer, Constants.cFlywheel::calculateRPM,
                                m_visionController::getFilteredDistance)));

        coController.rightTrigger.whenReleased(new RunCommand(() -> {
            m_flywheel.end();
            m_kickerWheel.end();
            m_indexer.end();
        }, m_flywheel, m_kickerWheel, m_indexer));

        coController.dPadLeft.whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.leftDeg));
        coController.dPadRight.whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.rightDeg));
        coController.dPadUp.whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.forwardDeg));
        coController.dPadDown.whenPressed(new TurretMotionMagic(m_turret, Constants.cTurret.backwardDeg));
    }

    public Command getAutonomousCommand() {
        RamseteCommand ramseteCommand = new RamseteCommand(sectorLineTo2Balls, m_drivetrain::getPose,
                new RamseteController(),
                new SimpleMotorFeedforward(Constants.cDrivetrain.kS, Constants.cDrivetrain.kV,
                        Constants.cDrivetrain.kA),
                Constants.cDrivetrain.kDriveKinematics, m_drivetrain::getWheelSpeeds, new PIDController(14.3, 0, 0),
                new PIDController(14.3, 0, 0), m_drivetrain::setVolts, m_drivetrain);
        return ramseteCommand.andThen(() -> m_drivetrain.end());
    }

    public void init() {
        m_drivetrain.init();
        m_flywheel.init();
        m_indexer.init();
        m_intake.init();
        m_kickerWheel.init();
        m_turret.init();
        m_visionController.init();
    }
}
