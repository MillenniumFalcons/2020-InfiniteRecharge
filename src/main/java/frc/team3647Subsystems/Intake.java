/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import lib.drivers.VictorSPXFactory;
import lib.wpi.Solenoid;

/**
 * Add your docs here.
 */
public class Intake implements PeriodicSubsystem {

    private final Solenoid innerLeftPiston;
    private final Solenoid innerRightPiston;
    private final Solenoid outerLeftPiston;
    private final Solenoid outerRightPiston;

    private final VictorSPX intakeMotor;

    public Intake(VictorSPXFactory.Configuration intakeMotorConfig, int leftPin1, int rightPin1, int leftPin2, int rightPin2) {
        if(intakeMotorConfig == null) {
            throw new IllegalArgumentException("Intake motor config was null");
        }
        intakeMotor = VictorSPXFactory.createVictor(intakeMotorConfig);
        innerLeftPiston = new Solenoid(leftPin1);
        innerRightPiston = new Solenoid(rightPin1);
        outerLeftPiston = new Solenoid(leftPin2);
        outerRightPiston = new Solenoid(rightPin2);
    }

    public void extendOuter() {
        setOuterPistons(false);
    }

    public void retractOuter() {
        setOuterPistons(false);
    }

    public void extendInner() {
        setInnerPistons(true);
    }

    public void retractInner() {
        setInnerPistons(false);
    }

    private void setInnerPistons(boolean on) {
        innerLeftPiston.set(on);
        innerRightPiston.set(on);
    }

    private void setOuterPistons(boolean on) {
        outerLeftPiston.set(on);
        outerRightPiston.set(on);
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

    @Override
    public String getName() {
        return "Intake";
    }
}
