/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.team3647Subsystems;

import lib.wpi.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * Add your docs here.
 */
public class BallIntake implements Subsystem {
    private Solenoid extensionCylinder1;
    private Solenoid extensionCylinder2;

    private VictorSPX intakeMotor;

    private boolean m_isInverted;
    private double m_timeStamp;
    private static BallShooter INSTANCE;

    private PeriodicIO periodicIO = new PeriodicIO();

    BallIntake(int extensionCylinder1Pin, int extensionCylinder2Pin, int intakeMotorPin,
            boolean isInverted) {
        extensionCylinder1 = new Solenoid(extensionCylinder1Pin);
        extensionCylinder2 = new Solenoid(extensionCylinder2Pin);
        intakeMotor = new VictorSPX(intakeMotorPin);
        m_isInverted = isInverted;
    }

    @Override
    public void init() {
        intakeMotor.setInverted(m_isInverted);
    }

    public class PeriodicIO {
        public double demand;
        public boolean solenoidPosition;
    }


    @Override
    public void writePeriodicOutputs() {
        intakeMotor.set(ControlMode.PercentOutput, periodicIO.demand);
        setPistons(periodicIO.solenoidPosition);
    }

    @Override
    /**
     * does nothing because it is an openloop subsystem.
     */
    public void readPeriodicInputs() {
    }

    @Override
    public void end() {
        setPistons(false);
        stop();
    }

    @Override
    public void periodic(double timestamp) {
        m_timeStamp = timestamp;
    }

    private void setPistons(boolean position) {
        extensionCylinder1.set(position);
        extensionCylinder2.set(position);
    }

    public void intake(double power) {
        setOpenLoop(power);
    }

    public void setOpenLoop(double demand) {
        periodicIO.demand = demand;
    }

    public void stop() {
        setOpenLoop(0);
    }

    @Override
    public String getName() {
        return "BallIntake";
    }
}
