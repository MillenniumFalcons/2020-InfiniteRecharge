package frc.robot;

public class Constants {

    public static class Drivetrain {
        //CAN id
        public static final int leftMasterPin = 1; 
        public static final int rightMasterPin = 1;
        public static final int leftSlavePin = 1;
        public static final int rightSlavePin = 1;

        public static final int stallCurrent = 35;
        public static final int maxCurrent = 60;

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
