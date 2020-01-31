/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;

public class launchSystem extends CommandBase {
  private Launcher launcher;
  private double indexSpeed;
  private double feedSpeed;
  private double launcherSpeed;
  private double maxSpeed;
  private double windSpeed;
  /**
   * Creates a new launchSystem.
   */
  public launchSystem(Launcher tLauncher, double IndexSpeed, double FeedSpeed, double launchSpeed) {
    launcher = tLauncher;
    indexSpeed = IndexSpeed;
    feedSpeed = FeedSpeed;
    maxSpeed = launchSpeed;
    windSpeed = Constants.windSpeedNEO;
    addRequirements(tLauncher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    launcher.feed(feedSpeed);
    launcher.index(indexSpeed);
    if(launcherSpeed < maxSpeed)
     {
        launcherSpeed += windSpeed*maxSpeed;//slowly increase the power to the shooter
     }
     launcher.launch(launcherSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLaunching();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
