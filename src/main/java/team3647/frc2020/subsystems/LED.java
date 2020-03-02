/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.frc2020.subsystems;

import com.ctre.phoenix.CANifier;

/**
 * Add your docs here.
 */
public class LED implements PeriodicSubsystem {

    private final CANifier m_CANifier;

    public LED(int canifierPin) {
        m_CANifier = new CANifier(canifierPin);
    }

    @Override
    public String getName() {
      return "LED";
    }
}
