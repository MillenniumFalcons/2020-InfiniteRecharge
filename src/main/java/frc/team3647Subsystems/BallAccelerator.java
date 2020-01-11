/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

/**
 * Add your docs here.
 */
public class BallAccelerator extends SparkMaxSubsystem {

    BallAccelerator(int masterCANID, double kEncoderAccelerationToUnits,
            double kEncoderVelocityToRPM, double kEncoderPositionToUnits, double[] PIDArr,
            double[] feedForwardArr, double maxVelocity, double maxAcceleration,
            double positionThreshold, double velocityThreshold, boolean inverted,
            int maxFreeSpeedCurrent, int maxStallCurrent) {
        super(masterCANID, kEncoderAccelerationToUnits, kEncoderVelocityToRPM,
                kEncoderPositionToUnits, PIDArr, feedForwardArr, maxVelocity, maxAcceleration,
                positionThreshold, velocityThreshold, inverted, maxFreeSpeedCurrent,
                maxStallCurrent);
    }

    public void setRPM(double RPM) {
        setVelocity(RPM);
    }

    @Override
    public String getName() {
        return "Ball Accelerator";
    }
}
