/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.Commands.AimTurret;
import frc.Commands.ArcadeDrive;
import frc.Commands.ExtendIntakeToGround;
import frc.Commands.GroundIntake;
import frc.Commands.LoadBalls;
import frc.Commands.LoadingStationIntake;
import frc.Commands.OrganizeFeeder;
import frc.Commands.RemoveBalls;
import frc.Commands.ShootContinuously;
import frc.Commands.StowIntakeAndOrganizeFeeder;
import frc.Commands.TurretManual;
import frc.Commands.TurretMotionMagic;
import frc.team3647Subsystems.Drivetrain;
import frc.team3647Subsystems.Flywheel;
import frc.team3647Subsystems.Indexer;
import frc.team3647Subsystems.Intake;
import frc.team3647Subsystems.KickerWheel;
import frc.team3647Subsystems.Turret;
import frc.team3647Subsystems.VisionController;
import frc.team3647inputs.Joysticks;
import lib.GroupPrinter;
import lib.IndexerSignal;
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
            Constants.cIndexer.rollersConfig, Constants.cIndexer.bannerSensorPin);

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
        // m_intake.extendInner();
        m_commandScheduler.registerSubsystem(m_kickerWheel, m_flywheel, m_visionController, m_intake, m_indexer,
                m_drivetrain, m_printer);
        m_commandScheduler.setDefaultCommand(m_drivetrain, new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
                mainController::getRightStickX, mainController.rightJoyStickPress::get));
        m_commandScheduler.setDefaultCommand(m_turret, new TurretManual(m_turret, coController::getLeftStickX));
        m_printer.addDouble("shooter rpm", m_flywheel::getVelocity);
        m_printer.addDouble("tunnel amps", this::getTunnelCurrent);
        m_printer.addDouble("funnel amps", this::getFunnelCurrent);
        configButtonBindings();
        // m_flywheel.setDefaultCommand(new RunCommand(() -> {
        // m_flywheel.setRPM(coController.rightTrigger.getTriggerValue() * 7000);
        // SmartDashboard.putNumber("required RPM",
        // coController.rightTrigger.getTriggerValue() * 7000);
        // SmartDashboard.putNumber("Shooter rpm", m_flywheel.getVelocity());
        // }, m_flywheel));

    }

    public double getZero() {
        return 0;
    }

    public boolean returnFalse() {
        return false;
    }

    private void configButtonBindings() {
        coController.leftTrigger.whenActive(new SequentialCommandGroup(new ConditionalCommand(new RunCommand(() -> {
            m_intake.retractInner();
        }, m_intake).withTimeout(.5), new InstantCommand(), m_intake::isInnerExtended),
                new ExtendIntakeToGround(m_intake).withTimeout(.25),
                new ParallelCommandGroup(new GroundIntake(m_intake, mainController::getLeftStickY),
                        new LoadBalls(m_kickerWheel, m_indexer))));

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

    private void setTestingCommands() {

        m_commandScheduler.registerSubsystem(m_flywheel);
        m_commandScheduler.registerSubsystem(m_indexer);
        m_commandScheduler.registerSubsystem(m_intake);
        m_commandScheduler.registerSubsystem(m_kickerWheel);
        SmartDashboard.putNumber("Shooter rpm", m_flywheel.getVelocity());
        SmartDashboard.putNumber("Shooter demand", 0);
        SmartDashboard.putNumber("Voltage", 0);
        SmartDashboard.putNumber("vertical Rollers current", pdp.getCurrent(10));
        SmartDashboard.putNumber("Kicker wheel current", pdp.getCurrent(11));
        // SmartDashboard.putNumber("raw velocity", 0);

        m_commandScheduler.setDefaultCommand(m_flywheel, new RunCommand(() -> {
            m_flywheel.setOpenloop(mainController.rightTrigger.getTriggerValue());
            SmartDashboard.putNumber("Shooter rpm", m_flywheel.getVelocity());
            SmartDashboard.putNumber("Shooter demand", mainController.rightTrigger.getTriggerValue());
            SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        }, m_flywheel));

        m_commandScheduler.setDefaultCommand(m_indexer, new RunCommand(() -> {
            m_indexer.set(new IndexerSignal(0, // mainController.getRightStickY(),
                    mainController.leftTrigger.getTriggerValue() * .7,
                    // 0,
                    mainController.leftTrigger.getTriggerValue()
            // 0
            ));
            SmartDashboard.putNumber("vertical Rollers currnet", pdp.getCurrent(10));
            // mainController.getRightStickY(), mainController.getRightStickY()));
        }, m_indexer));

        // m_commandScheduler.setDefaultCommand(m_intake, new RunCommand(() -> {
        // m_intake.intake(mainController.rightTrigger.getTriggerValue());
        // }, m_intake));

        m_commandScheduler.setDefaultCommand(m_kickerWheel, new RunCommand(() -> {
            m_kickerWheel.setOpenloop(mainController.rightTrigger.getTriggerValue());
            SmartDashboard.putNumber("Kicker wheel currnet", pdp.getCurrent(11));
        }, m_kickerWheel));
    }

    public void setTestingIndexer() {
        m_commandScheduler.registerSubsystem(m_kickerWheel, m_indexer, m_intake);
        mainController.leftBumper.whenActive(new ParallelCommandGroup(new RunCommand(() -> {
            // m_intake.intake(.6);
        }, m_intake), new LoadBalls(m_kickerWheel, m_indexer)));

        mainController.rightBumper.whenActive(new RunCommand(() -> {
            m_indexer.set(IndexerSignal.INDEXERBACK);
        }, m_indexer));

        mainController.rightBumper.whenReleased(new RunCommand(() -> {
            m_indexer.end();
        }, m_indexer));

        mainController.leftBumper.whenReleased(new OrganizeFeeder(m_indexer, m_kickerWheel).withTimeout(3));
        mainController.leftBumper.whenReleased(new RunCommand(() -> {
            m_intake.intake(0);
        }));

        // mainController.leftBumper.whenActive(new RunCommand(() -> {
        // m_intake.intake(.6);
        // }));
    }

    public void setTurretTesting() {
        m_commandScheduler.registerSubsystem(m_turret, m_visionController);
        airCompressor.stop();
        SmartDashboard.putNumber("turret encoder", m_turret.getPosition());
        SmartDashboard.putBoolean("turret sensor", m_turret.isOnLimitSwitch());
        m_commandScheduler.setDefaultCommand(m_turret, new RunCommand(() -> {
            // m_turret.setOpenloop(mainController.getLeftStickY() * .5);
            SmartDashboard.putNumber("turret encoder", m_turret.getPosition());
            SmartDashboard.putNumber("turret Velocity", m_turret.getVelocity());
            SmartDashboard.putBoolean("turret sensor", m_turret.isOnLimitSwitch());
        }, m_turret));

        m_commandScheduler.setDefaultCommand(m_visionController, new RunCommand(() -> {
            SmartDashboard.putNumber("angle to target", m_visionController.getFilteredYaw());
            SmartDashboard.putNumber("distance to target",
                    Units.metersToFeet(m_visionController.getFilteredDistance()));
        }, m_visionController));

        mainController.dPadUp.whenActive(new TurretMotionMagic(m_turret, 0));
        mainController.dPadLeft.whenActive(new TurretMotionMagic(m_turret, 90));
        mainController.dPadRight.whenActive(new TurretMotionMagic(m_turret, -90));

        mainController.rightBumper.whenActive(new AimTurret(m_turret, m_visionController::getFilteredYaw));
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

    private double getTunnelCurrent() {
        return pdp.getCurrent(8);
    }

    private double getFunnelCurrent() {
        return pdp.getCurrent(5);
    }
}
