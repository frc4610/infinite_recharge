/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double groundToPowerPortIn = 98.25;
    public static double groundToLimeLensIn = 27.25;
    public static double groundToLimeLensDeg = 15;
    public static double groundToLimeLensRad = Math.toRadians(groundToLimeLensDeg);
    public static double windSpeedNEO = .04;//increases by this speed once per 20ms
    public static double feedNEOSpeed = .5;
    public static double launchNEOSpeed = 1;//eventually change to be calculated by vision systems
    public static double indexNEOSpeed = .5;
    public static float kp = .04f;
    public static float minPower = .01f;
}
