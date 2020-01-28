package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import lib.drivers.SparkMaxFactory;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;

public class Constants {

        public static class Drivetrain {
                // CAN id
                public static final int leftMasterPin = 1;
                public static final int rightMasterPin = 2;
                public static final int leftSlavePin = 3;
                public static final int rightSlavePin = 4;
                public static final int stallCurrent = 35;
                public static final int maxCurrent = 60;

                SparkMaxFactory.Configuration leftMasterConfig =
                                new SparkMaxFactory.Configuration(leftMasterPin, true)
                                                .currentLimiting(true, maxCurrent, stallCurrent)
                                                .idleMode(IdleMode.kBrake)
                                                .voltageCompensation(true, 12.0);

                SparkMaxFactory.Configuration rightMasterMasterConfig =
                                new SparkMaxFactory.Configuration(rightMasterPin, true)
                                                .currentLimiting(true, maxCurrent, stallCurrent)
                                                .idleMode(IdleMode.kBrake)
                                                .voltageCompensation(true, 12.0);

                SparkMaxFactory.Configuration leftSlaveConfig = SparkMaxFactory.Configuration
                                .mirrorWithCANID(leftMasterConfig, leftSlavePin);

                SparkMaxFactory.Configuration rightSlaveConfig = SparkMaxFactory.Configuration
                                .mirrorWithCANID(rightMasterMasterConfig, rightSlavePin);


                // meters per second
                public static final double maxVelocity = 3.2;

                // meters (6inches)
                public static final double wheelDiameter = .1524;

        }

        public static class BallIntake {
                public static final int solenoidPin1 = 0;
                public static final int solenoidPin2 = 3;
                public static final int intakeMotorPin = 2;
        }

        /**
         * In order to make the class not be able to be an object
         */
        private Constants() {
        }
}
