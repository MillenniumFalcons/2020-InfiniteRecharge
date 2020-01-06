/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import lib.wpi.Solenoid;

/**
 * Add your docs here.
 */
public class BallShooter extends TalonSRXSubsystem {

    private static BallShooter INSTANCE;
    private double kRPMToEncoderVelocity;
    private VictorSPX victorFollower;
    private Solenoid hoodPiston;

    BallShooter(int masterCANID, double[] PIDArr, double[] feedForwardArr, int maxVelocity,
            int maxAcceleration, double encoderThreshold, int maxStallCurrent, int maxCurrent,
            int peakCurrentDuration, boolean inverted, double rpmToEncoderVelocity,
            int followerCANID, int hoodPistonPin) {
        super(masterCANID, PIDArr, feedForwardArr, maxVelocity, maxAcceleration, encoderThreshold,
                maxStallCurrent, maxCurrent, peakCurrentDuration, inverted);
        kRPMToEncoderVelocity = rpmToEncoderVelocity;
        victorFollower = new VictorSPX(followerCANID);
        hoodPiston = new Solenoid(hoodPistonPin);
    }

    @Override
    public void init() {
        super.init();
        addFollower(victorFollower);
    }

    public void setRPM(double RPM) {
        setVelocity(RPM * kRPMToEncoderVelocity);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
    }

    @Override
    public void writePeriodicOutputs() {
        super.writePeriodicOutputs();
    }




    @Override
    public String getName() {
        return "BallShooter";
    }
}
