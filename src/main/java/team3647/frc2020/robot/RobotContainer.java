/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.robot;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2020.autonomous.Trajectories;
import team3647.frc2020.autonomous.Trajectories;
import team3647.frc2020.commands.AimTurret;
import team3647.frc2020.commands.ArcadeDrive;
import team3647.frc2020.commands.AutoAimTurretHood;
import team3647.frc2020.commands.BatterShot;
import team3647.frc2020.commands.ExtendIntakeToGround;
import team3647.frc2020.commands.GroundIntake;
import team3647.frc2020.commands.IndexerManual;
import team3647.frc2020.commands.InitiationLineShot;
import team3647.frc2020.commands.LoadBalls;
import team3647.frc2020.commands.LoadingStationIntake;
import team3647.frc2020.commands.MoveHood;
import team3647.frc2020.commands.OrganizeFeeder;
import team3647.frc2020.commands.RemoveBalls;
import team3647.frc2020.commands.ShootClosedLoop;
import team3647.frc2020.commands.ShootOpenloop;
import team3647.frc2020.commands.StopShooting;
import team3647.frc2020.commands.StowIntakeAndOrganizeFeeder;
import team3647.frc2020.commands.TrenchShot;
import team3647.frc2020.commands.TurretManual;
import team3647.frc2020.commands.TurretMotionMagic;
import team3647.frc2020.subsystems.BallStopper;
import team3647.frc2020.subsystems.Drivetrain;
import team3647.frc2020.subsystems.Flywheel;
import team3647.frc2020.subsystems.Hood;
import team3647.frc2020.subsystems.Indexer;
import team3647.frc2020.subsystems.Intake;
import team3647.frc2020.subsystems.KickerWheel;
import team3647.frc2020.subsystems.LED;
import team3647.frc2020.subsystems.Turret;
import team3647.frc2020.subsystems.VisionController;
import team3647.frc2020.inputs.Joysticks;
import team3647.frc2020.inputs.Limelight.LEDMode;
import team3647.lib.DriveSignal;
import team3647.lib.GroupPrinter;
import team3647.lib.IndexerSignal;
import team3647.lib.wpi.Compressor;
import team3647.lib.wpi.PDP;

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

    private final Indexer m_indexer = new Indexer(Constants.cIndexer.leftRollersConfig,
            Constants.cIndexer.rightRollersConfig, Constants.cIndexer.tunnelConfig,
            Constants.cIndexer.horizontalRollersConfig, Constants.cIndexer.bannerSensorPin, pdp::getCurrent);

    private final Intake m_intake = new Intake(Constants.cIntake.intakeMotorConfig, Constants.cIntake.innerPistonsPin,
            Constants.cIntake.outerPistonsPin);

    private final KickerWheel m_kickerWheel = new KickerWheel(Constants.cKickerWheel.masterConfig,
            Constants.cKickerWheel.pidConfig);

    private final Turret m_turret = new Turret(Constants.cTurret.masterConfig, Constants.cTurret.pidConfig,
            Constants.cTurret.kMaxRotationDeg, Constants.cTurret.kMinRotationDeg, Constants.cTurret.limitSwitchPin);

    private final Hood m_hood = new Hood(Constants.cHood.pwmPort, Constants.cHood.minPosition,
            Constants.cHood.maxPosition);

    private final BallStopper m_ballStopper = new BallStopper(Constants.cBallStopper.solenoidPin);

    private final GroupPrinter m_printer = GroupPrinter.getInstance();

    private final LED m_LED = new LED(Constants.cLED.canifierPin);

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

    public RobotContainer() {
        // setTestingIndexer();
        airCompressor.start();
        pdp.clearStickyFaults();
        m_intake.extendInner();
        m_commandScheduler.registerSubsystem(m_kickerWheel, m_flywheel, m_visionController, m_intake, m_indexer,
                m_drivetrain, m_hood, m_ballStopper, m_printer, m_LED);

        m_commandScheduler.setDefaultCommand(m_drivetrain, new ArcadeDrive(m_drivetrain, mainController::getLeftStickY,
                mainController::getRightStickX, mainController.rightJoyStickPress::get, mainController.buttonA::get));
        m_commandScheduler.setDefaultCommand(m_turret, new TurretManual(m_turret, coController::getLeftStickX));
        m_indexer.setDefaultCommand(new IndexerManual(m_indexer, coController::getRightStickY));

        m_LED.setDefaultCommand(new RunCommand(() -> {
            m_LED.set(SmartDashboard.getNumber("led - red", .5), SmartDashboard.getNumber("led - green", .5),
                    SmartDashboard.getNumber("led - blue", .5));
        }, m_LED));

        SmartDashboard.putNumber("led - red", 1);
        SmartDashboard.putNumber("led - green", 0);
        SmartDashboard.putNumber("led - blue", 0);

        m_printer.addDouble("tunnel amps", () -> {
            return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
        });
        m_printer.addDouble("funnel amps", () -> {
            return pdp.getCurrent(m_indexer.getTunnelPDPSlot());
        });
        m_printer.addDouble("shooter rpm based on distance", this::getFlywheelRPM);
        m_printer.addDouble("kicker wheel amps", m_kickerWheel::getMasterCurrent);
        m_printer.addDouble("hoodPosition", m_hood::getAppliedPosition);
        m_printer.addDouble("turret distance to target", m_visionController::getFilteredDistance);
        m_printer.addDouble("shooter velocity", m_flywheel::getVelocity);
        m_printer.addDouble("drivetrain left velocity", m_drivetrain::getLeftVelocity);
        m_printer.addDouble("heading", m_drivetrain::getHeading);

        configButtonBindings();
    }

    private void configButtonBindings() {
        coController.leftTrigger.whenActive(new SequentialCommandGroup(new RunCommand(() -> {
            m_intake.retractInner();
        }, m_intake).withTimeout(.5), new ExtendIntakeToGround(m_intake).withTimeout(.25), new ParallelCommandGroup(
                new GroundIntake(m_intake, mainController::getLeftStickY), new LoadBalls(m_indexer, m_ballStopper))));

        coController.leftTrigger
                .whenReleased(new StowIntakeAndOrganizeFeeder(m_intake, m_indexer, m_kickerWheel).withTimeout(3));

        coController.leftBumper.whenActive(
                new ParallelCommandGroup(new LoadingStationIntake(m_intake), new LoadBalls(m_indexer, m_ballStopper)));
        coController.leftBumper.whenReleased(new OrganizeFeeder(m_indexer, m_kickerWheel).withTimeout(3));

        coController.buttonA.whenActive(new RemoveBalls(m_indexer, m_intake, m_kickerWheel));
        coController.buttonA.whenReleased(new RunCommand(() -> {
            m_intake.end();
            m_indexer.end();
            m_kickerWheel.end();
            m_intake.retractOuter();
            m_intake.extendInner();
        }, m_indexer, m_intake, m_kickerWheel).withTimeout(.5));

        // coController.rightBumper.whenActive(new AutoAimTurretHood(m_hood, m_turret,
        // () -> {
        // return
        // Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        // }, m_visionController::getFilteredYaw));

        coController.rightBumper.whenActive(new AimTurret(m_turret, m_visionController::getFilteredYaw));

        coController.rightTrigger.whenActive(new ParallelCommandGroup(
                new AutoAimTurretHood(m_hood, m_turret, this::getHoodPosition, m_visionController::getFilteredYaw),
                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper, this::getFlywheelRPM,
                        Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM, IndexerSignal.GO_FAST)));

        coController.leftJoyStickPress.whenActive(new TurretManual(m_turret, coController::getLeftStickX));

        coController.buttonB.whenActive(new ParallelCommandGroup(new MoveHood(m_hood, Constants.cHood.cpShotPosition),
                new TrenchShot(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper)));

        coController.buttonY
                .whenActive(new ParallelCommandGroup(new MoveHood(m_hood, Constants.cHood.rightUpToTowerShotPosition),
                        new BatterShot(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper)));

        // coController.rightTrigger.whenActive(
        // new TrenchShot(m_flywheel, m_kickerWheel, m_indexer)
        // );

        coController.rightTrigger.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

        coController.buttonB.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

        coController.buttonY.whenReleased(new StopShooting(m_flywheel, m_kickerWheel, m_indexer));

        // coController.dPadLeft.whenPressed(
        // new TurretMotionMagic(m_turret, Constants.cTurret.leftDeg).withTimeout(.5));
        // coController.dPadRight.whenPressed(
        // new TurretMotionMagic(m_turret, Constants.cTurret.rightDeg).withTimeout(.5));
        // coController.dPadUp.whenPressed(
        // new TurretMotionMagic(m_turret,
        // Constants.cTurret.forwardDeg).withTimeout(.5));
        // coController.dPadDown.whenPressed(
        // new TurretMotionMagic(m_turret,
        // Constants.cTurret.backwardDeg).withTimeout(.5));

        coController.dPadLeft.whenPressed(new RunCommand(() -> {
            m_hood.setPosition(0.2);
        }, m_hood));

        coController.dPadRight.whenPressed(new RunCommand(() -> {
            m_hood.setPosition(.645);
        }, m_hood));

        coController.dPadDown.whenPressed(new RunCommand(() -> {
            m_hood.setPosition(.7);
        }, m_hood));

        coController.dPadUp.whenPressed(new RunCommand(() -> {
            m_hood.setPosition(.8);
        }, m_hood));
    }

    public void init() {
        m_drivetrain.init();
        m_drivetrain.setOdometry(Trajectories.initiationLineToTrenchBalls.getInitialPose(), new Rotation2d());
        m_flywheel.init();
        m_indexer.init();
        m_intake.init();
        m_kickerWheel.init();
        m_turret.init();
        m_visionController.init();
        m_visionController.setLedMode(LEDMode.ON);
    }

    public Command getAutonomousCommand() {
        RamseteCommand command1 = new RamseteCommand(Trajectories.initiationLineToTrenchBalls, m_drivetrain::getPose,
                new RamseteController(), Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS,
                m_drivetrain);
        RamseteCommand command2 = new RamseteCommand(Trajectories.trenchBall3ToFrontOfTower, m_drivetrain::getPose,
                new RamseteController(), Constants.cDrivetrain.kDriveKinematics, m_drivetrain::setVelocityMpS,
                m_drivetrain);

        Command intakeCommand = new SequentialCommandGroup(new RunCommand(() -> {
            m_intake.retractInner();
        }, m_intake).withTimeout(.5), new ExtendIntakeToGround(m_intake).withTimeout(.25),
                new ParallelCommandGroup(new GroundIntake(m_intake, () -> {
                    return .8;
                }), new LoadBalls(m_indexer, m_ballStopper)));

        Command aimCommand = new AutoAimTurretHood(m_hood, m_turret, () -> {
            return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        }, m_visionController::getFilteredYaw);
        Command aimCommand2 = new AutoAimTurretHood(m_hood, m_turret, () -> {
            return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        }, m_visionController::getFilteredYaw);

        Command shootCommand = new ParallelCommandGroup(new AutoAimTurretHood(m_hood, m_turret, () -> {
            return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        }, m_visionController::getFilteredYaw),
                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper, () -> {
                    return Constants.cFlywheel.getFlywheelRPM(m_visionController.getFilteredDistance());
                }, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM, IndexerSignal.GO_FAST));

        Command shootCommand2 = new ParallelCommandGroup(new AutoAimTurretHood(m_hood, m_turret, () -> {
            return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
        }, m_visionController::getFilteredYaw),
                new ShootClosedLoop(m_flywheel, m_kickerWheel, m_indexer, m_ballStopper, () -> {
                    return Constants.cFlywheel.getFlywheelRPM(m_visionController.getFilteredDistance());
                }, Constants.cKickerWheel::getFlywheelOutputFromFlywheelRPM, IndexerSignal.GO_FAST));

        return new SequentialCommandGroup(aimCommand.withTimeout(.4), shootCommand.withTimeout(4),
                new ParallelRaceGroup(command1, intakeCommand), command2, new RunCommand(() -> {
                    m_drivetrain.setOpenLoop(DriveSignal.BRAKE);
                }, m_drivetrain), aimCommand2.withTimeout(.5), shootCommand2);
    }

    public void stopDrivetrain() {
        m_drivetrain.setOpenLoop(DriveSignal.BRAKE);
    }

    private double getHoodPosition() {
        return Constants.cHood.getHoodPosition(m_visionController.getFilteredDistance());
    }

    private double getFlywheelRPM() {
        return Constants.cFlywheel.getFlywheelRPM(m_visionController.getFilteredDistance());
    }
}