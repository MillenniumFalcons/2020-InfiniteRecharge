/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3647inputs;

/**
 * Camera Class to get "working distance" or "projected distance" using the following constants and variables
 */
public class PS3EyeCamera 
{
    public double goalHeight = 2.438; //8ft in meters
    public double cameraHeight = 0.762; //30inches in meters

    public final double FOV = 75; //can be either 75 or 56 degrees depending on the lens setting used
    public final double SENSORHEIGHT = 2952; //micro meters
    public final double SENSORWIDTH = 3984; //micro meters
    public final double PIXELSIZE = 6; //6 micrometers per pixel


    public double focalLength = ((SENSORWIDTH / 2) * (1 / Math.tan(Math.toRadians(FOV/2)))) / 1000000; //gives focal length in meters (divide by 1000000 for micrometers to meters)


    public double getProjectedDistance(double pitch)
    {
        return ((goalHeight - cameraHeight) * focalLength) / pitch;
    }

    public double getPhi(double pitch)
    {
        return Math.atan(pitch / focalLength);
    }


}
