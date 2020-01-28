package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import lib.drivers.SparkMaxFactory;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.team3647Utils.RGB;

public class Constants {

        public static class Drivetrain {
                // CAN id
                public static final int leftMasterPin = 1;
                public static final int leftSlavePin = 2;
                public static final int rightMasterPin = 3;
                public static final int rightSlavePin = 4;
                public static final int stallCurrent = 35;
                public static final int maxCurrent = 60;

                // meters per second
                public static final double maxVelocity = 3.2;

                // meters (6inches)
                public static final double wheelDiameter = .1524;

                public static final double gearboxReduction = 9.0 / 40.0 * 24.0 / 42.0;

                public static final SparkMaxFactory.Configuration leftMasterConfig = new SparkMaxFactory.Configuration(
                                leftMasterPin, false).currentLimiting(true, maxCurrent, stallCurrent)
                                                .idleMode(IdleMode.kBrake).voltageCompensation(true, 12.0);

                public static final SparkMaxFactory.Configuration rightMasterMasterConfig = new SparkMaxFactory.Configuration(
                                rightMasterPin, true).currentLimiting(true, maxCurrent, stallCurrent)
                                                .idleMode(IdleMode.kBrake).voltageCompensation(true, 12.0);

                public static final SparkMaxFactory.Configuration leftSlaveConfig = SparkMaxFactory.Configuration
                                .mirrorWithCANID(leftMasterConfig, leftSlavePin);

                public static final SparkMaxFactory.Configuration rightSlaveConfig = SparkMaxFactory.Configuration
                                .mirrorWithCANID(rightMasterMasterConfig, rightSlavePin);

                public static final ClosedLoopConfig leftMasterPIDConfig = new ClosedLoopConfig()
                                .encoderVelocityToRPM(gearboxReduction);
                public static final ClosedLoopConfig rightMasterPIDConfig = new ClosedLoopConfig()
                                .encoderVelocityToRPM(gearboxReduction);

        }

    public static class BallIntake {
        public static final int solenoidPin1 = 0;
        public static final int solenoidPin2 = 3;
        public static final int intakeMotorPin = 2;
    }

    public static class cPPSpinner {
        public static final RGB red = new RGB(new double[] { .51, .35, .14 });
        public static final RGB green = new RGB(new double[] { .15, .59, .25 });
        public static final RGB blue = new RGB(new double[] { .12, .42, .45 });
        public static final RGB yellow = new RGB(new double[] { .32, .56, .12 });
        // public static final RGB test = new RGB(new double[] {});
        public static final double colorThreshold = 0.05; // percentage
    }

    /**
     * In order to make the class not be able to be an object
     */
    private Constants() {
    }
}
