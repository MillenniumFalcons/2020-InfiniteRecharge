/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import lib.drivers.TalonSRXFactory;
import lib.drivers.VictorSPXFactory;
import lib.wpi.Solenoid;

/**
 * Intake fuel cells from everywhere.
 */
public class Intake implements PeriodicSubsystem {

    private final Solenoid innerPistons;
    private final Solenoid outerPistons;

    private final TalonSRX intakeMotor;
    private boolean isInnerExtended = false;

    public Intake(TalonSRXFactory.Configuration intakeMotorConfig, int innerPistonsPin,
            int outerPistonsPin) {
        if (intakeMotorConfig == null) {
            throw new NullPointerException("Intake motor config was null");
        }
        intakeMotor = TalonSRXFactory.createTalon(intakeMotorConfig);
        innerPistons = new Solenoid(innerPistonsPin);
        outerPistons = new Solenoid(outerPistonsPin);
    }

    @Override
    public void periodic() {
        isInnerExtended = innerPistons.get();
    }

    public void extendOuter() {
        outerPistons.set(true);
    }

    public void retractOuter() {
        outerPistons.set(false);
    }

    public void extendInner() {
        innerPistons.set(true);
    }

    public void retractInner() {
        innerPistons.set(false);
    }

    private void setOpenLoop(double demand) {
        intakeMotor.set(ControlMode.PercentOutput, demand);
    }

    public void intake(double demand) {
        setOpenLoop(-demand);
    }

    public void spitOut(double demand) {
        setOpenLoop(demand);
    }

    public boolean isInnerExtended() {
        return isInnerExtended;
    }

    @Override
    public void end() {
        setOpenLoop(0);
    }

    @Override
    public String getName() {
        return "Intake";
    }
}
