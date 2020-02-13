/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class autoLaunchThenDrive extends SequentialCommandGroup {
  /**
   * Creates a new autoLaunchThenDrive.
   */
  public autoLaunchThenDrive() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new vLED(RobotContainer.visionSensor, true),
      new visionTarget(RobotContainer.visionSensor, RobotContainer.driveBase, RobotContainer.launcher, true),
      new vLED(RobotContainer.visionSensor, false),
      new encoderMovement(RobotContainer.driveBase, RobotContainer.mainEncoders, RobotContainer.gyro, 12));
  }
}
