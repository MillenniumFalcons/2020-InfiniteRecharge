/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package lib.team3647Utils;

/**
 * Add your docs here.
 */
public class TimestampedVisionTarget extends TimestampedData {
    private final VisionTarget m_target;

    public TimestampedVisionTarget(double timestamp, VisionTarget target) {
        super(timestamp);
        m_target = target;
    }

    /**
     * @return the m_target
     */
    public VisionTarget getTarget() {
        return m_target;
    }


}
