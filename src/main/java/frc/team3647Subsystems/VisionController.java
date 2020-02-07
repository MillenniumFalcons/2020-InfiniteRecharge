/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import java.util.Objects;
import frc.team3647inputs.Limelight;
import frc.team3647inputs.Limelight.Data;
import lib.team3647Utils.VisionTarget;
import lib.wpi.HALMethods;

/**
 * give u angles that u need
 */
public class VisionController implements PeriodicSubsystem {

    public static class CamConstants {
        public final double kGoalHeight;
        public final double kCameraHeight;
        public final double kCameraAngle;

        public CamConstants(double goalHeight, double cameraHeight, double cameraAngle) {
            this.kGoalHeight = goalHeight;
            this.kCameraHeight = cameraHeight;
            this.kCameraAngle = cameraAngle;
        }
    }

    public static class PeriodicIO {
        public boolean validTarget;
        public double x;
        public double y;
        public double area;
        public double skew;
        public double latency;
        public double range;

        public Limelight.LEDMode ledMode = Limelight.LEDMode.DEFAULT;
        public Limelight.CamMode camMode = Limelight.CamMode.VISION;
        public Limelight.StreamMode streamMode = Limelight.StreamMode.Limelight;
    }

    private final Limelight limelight;
    private final CamConstants m_constants;
    private VisionTarget latestTarget;
    private boolean outputsHaveChanged = false;
    private PeriodicIO periodicIO = new PeriodicIO();

    public VisionController(String camIP, CamConstants constants) {
        Objects.requireNonNull(camIP);
        Objects.requireNonNull(constants);
        limelight = new Limelight(camIP);
        m_constants = constants;

    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.validTarget = (int) limelight.get(Data.VALID_TARGET) == 1;
        periodicIO.x = limelight.get(Data.X);
        periodicIO.y = limelight.get(Data.Y);
        periodicIO.area = limelight.get(Data.AREA);
        periodicIO.skew = limelight.get(Data.SKEW);
        periodicIO.latency = limelight.get(Data.LATNECY);
        periodicIO.range = calculateRange(periodicIO.y);
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsHaveChanged) {
            limelight.camMode(periodicIO.camMode);
            limelight.ledMode(periodicIO.ledMode);
            limelight.streamMode(periodicIO.streamMode);
            HALMethods.sendDSWarning("limelight changed modes!");
            outputsHaveChanged = false;
        }
    }

    @Override
    public void periodic() {

    }

    public double getDistance() {
        return latestTarget.getDistance();
    }

    public double getYaw() {
        return latestTarget.getX();
    }

    public boolean isValid() {
        return latestTarget.getValid() == 1;
    }

    public double getPitch() {
        return latestTarget.getY();
    }

    private double calculateRange(double yDegrees) {
        return (m_constants.kGoalHeight - m_constants.kCameraHeight)
                / Math.tan(Math.toRadians(yDegrees + m_constants.kCameraAngle));
    }

    public void setCamMode(Limelight.CamMode mode) {
        periodicIO.camMode = mode;
        outputsHaveChanged = true;
    }

    public void setStreamMode(Limelight.StreamMode mode) {
        periodicIO.streamMode = mode;
        outputsHaveChanged = true;
    }

    public void setLedMode(Limelight.LEDMode mode) {
        periodicIO.ledMode = mode;
        outputsHaveChanged = true;
    }

    @Override
    public String getName() {
        return "VisionController";
    }
}
