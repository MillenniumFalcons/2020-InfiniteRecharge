/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Add your docs here.
 */
public class ClosedLoopFactory {
    public static class ClosedLoopConfig {
        public final double kEncoderAccelerationToUnits;
        public final double kEncoderVelocityToRPM;
        public final double kEncoderTicksToUnits;
        public final double kP, kI, kD;
        public final double kS, kV, kA;
        public final double maxVelocity;
        public final double maxAcceleration;
        public final double positionThreshold;
        public final double velocityThreshold;

        private ClosedLoopConfig() {
            kEncoderAccelerationToUnits = 1;
            kEncoderVelocityToRPM = 1;
            kEncoderTicksToUnits = 1;
            kP = 0;
            kI = 0;
            kD = 0;
            kS = 0;
            kV = 0;
            kA = 0;
            maxVelocity = 1;
            maxAcceleration = 1;
            positionThreshold = 1;
            velocityThreshold = 1;
        }

        public ClosedLoopConfig(double kEncoderAccelerationToUnits, double kEncoderVelocityToRPM,
                double kEncoderTicksToUnits, double kP, double kI, double kD, double kS, double kV,
                double kA, double[] feedForwardArr, double maxVelocity, double maxAcceleration,
                double positionThreshold, double velocityThreshold) {
            this.kEncoderAccelerationToUnits = kEncoderAccelerationToUnits;
            this.kEncoderVelocityToRPM = kEncoderVelocityToRPM;
            this.kEncoderTicksToUnits = kEncoderTicksToUnits;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.positionThreshold = positionThreshold;
            this.velocityThreshold = velocityThreshold;
        }
    }

    public static ClosedLoopConfig DEFAULT = new ClosedLoopConfig();

    private static void handleCANError(int id, CANError error, String message) {
        if (error != CANError.kOk) {
            DriverStation.reportError("Could not configure spark id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    public static CANPIDController createSparkMaxPIDController(CANSparkMax master,
            CANEncoder feedbackDevice, ClosedLoopConfig config, int slot) {
        CANPIDController controller = master.getPIDController();
        configSparkMaxPIDController(controller, master, feedbackDevice, config, slot);
        return controller;
    }

    public static void configSparkMaxPIDController(CANPIDController controller, CANSparkMax master,
            CANEncoder feedbackDevice, ClosedLoopConfig config, int slot) {
        int id = master.getDeviceId();
        double maxVelocityTicks = config.maxVelocity / config.kEncoderVelocityToRPM;
        double maxAccelerationTicks = config.maxVelocity / config.kEncoderAccelerationToUnits;
        handleCANError(id, controller.setP(config.kP, slot), "set P");
        handleCANError(id, controller.setI(config.kI, slot), "set I");
        handleCANError(id, controller.setD(config.kD, slot), "set D");
        handleCANError(id, controller.setSmartMotionMaxAccel(maxAccelerationTicks, slot),
                "set smart motion accel");
        handleCANError(id, controller.setSmartMotionMaxVelocity(maxVelocityTicks, slot),
                "set smart motion max vel");
        handleCANError(id, controller.setSmartMotionMinOutputVelocity(-maxVelocityTicks, slot),
                "set smart motion min vel");
        handleCANError(id, controller.setFeedbackDevice(feedbackDevice), "set feedback device");
    }


    private static void handleCANError(int id, ErrorCode error, String message) {
        if (error != ErrorCode.OK) {
            DriverStation.reportError("Could not configure talon id: " + id + " error: "
                    + error.toString() + " " + message, false);
        }
    }

    public static void configTalonPIDController(TalonSRX talon, FeedbackDevice feedbackDevice,
            ClosedLoopConfig config, int slot) {
        int id = talon.getDeviceID();
        int maxVelocityTicks = (int)(config.maxVelocity / config.kEncoderVelocityToRPM);
        int maxAccelerationTicks = (int)(config.maxVelocity / config.kEncoderAccelerationToUnits);
        handleCANError(id, talon.config_kP(slot, config.kP), "set kP");
        handleCANError(id, talon.config_kI(slot, config.kI), "set kI");
        handleCANError(id, talon.config_kD(slot, config.kD), "set kD");
        handleCANError(id, talon.configMotionAcceleration(maxAccelerationTicks),
                "set acceleration");
        handleCANError(id, talon.configMotionCruiseVelocity(maxVelocityTicks),
                "set velocity");
        handleCANError(id, talon.configSelectedFeedbackSensor(feedbackDevice, slot, 0),
                "config feedback device");

    }
}