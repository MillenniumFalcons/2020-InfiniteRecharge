/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.revrobotics.CANSparkMax;
import lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import lib.drivers.SparkMaxFactory;

/**
 * Shoots ball.
 */
public class Flywheel extends SparkMaxSubsystem {

    private CANSparkMax follower;

    public Flywheel(SparkMaxFactory.Configuration masterConfig,
            SparkMaxFactory.Configuration followerConfig,
            ClosedLoopConfig pidConfig) {
        super(masterConfig, pidConfig);
        follower = addFollower(followerConfig, false);

    }

    public void setRPM(double RPM) {
        setVelocity(RPM);
    }


    @Override
    public String getName() {
        return "Flywheel";
    }
}
