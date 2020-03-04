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
    public static double middleIntakeEncoderPosition = -1205;
    public static double bottomIntakeEncoderPosition = -1500;
    public static double groundToPowerPortIn = 98.25;
    public static double groundToLimeLensIn = 18.75;
    public static double groundToLimeLensDeg = 16.641;
    public static double groundToLimeLensRad = Math.toRadians(groundToLimeLensDeg);
    public static double distanceToPowerportMaxIn = (23*12);//temporary change, fix to 23 feet later
    public static double distanceToPowerportMinIn = (10*12);//temporary change, fix to 10 feet later
    public static double windSpeedNEO = .06;//increases by this proportion, max*wind,  once per 20ms
    public static double feedDelay = 1.9;
    public static double autoLaunchDelay = 9;
    public static double feedNEOSpeed = .90;
    public static double baselineLaunchSpeedLower = .44;
    public static double baselineLaunchSpeedHigher = .214;
    public static double launchNEOSpeed = .74;
    public static double indexNEOSpeed = .8;
    public static double kp = .02;
    public static double minPower = .04;
    public static double gravityFeetSeconds = 32.17405;
    public static double gravityInchesSeconds = 12*gravityFeetSeconds;
    public static double heightDifferenceLauncher = 76.25;
    public static double launchAngleDeg = 60;
    public static double launchAngleRad = 1.0472;
    public static double launchWheelRadius = 3;
    public static double maxFalconVelocity = 22040;
    public static double launchMaxVelocity = 5650;
    public static double overheatValue = 50;
}
