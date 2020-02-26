/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Launcher;



public class launchSystem extends CommandBase {
  private Launcher launcher;
  private double indexSpeed;
  private double feedSpeed;
  private Timer timer;
  private boolean isAuto;
  private boolean previousState;
  private Timer feedTimer;
  /**
   * Creates a new launchSystem.
   */
  public launchSystem(Launcher tLauncher, double IndexSpeed, double FeedSpeed, boolean auto) {
    launcher = tLauncher;
    indexSpeed = IndexSpeed;
    feedSpeed = FeedSpeed;
    timer = new Timer();
    previousState = false;
    isAuto = auto;
    feedTimer = new Timer();
    addRequirements(tLauncher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!RobotContainer.stateOfFeed())
    {
      previousState = true;
    }
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > Constants.feedDelay)
    {
      /*if(RobotContainer.stateOfFeed() && !previousState)
      {
        feedTimer.start();
      }
      else if (!RobotContainer.stateOfFeed() && previousState)
      {
        feedTimer.reset();
      }
      else if(!RobotContainer.stateOfFeed() && !previousState)
      {
        feedTimer.start();
      }

      previousState = RobotContainer.stateOfFeed();

      if(feedTimer.get() >= .25)
      {
        launcher.feed(feedSpeed);
      }
      else
      {
        launcher.feed(0);
      }*/

      launcher.feed(feedSpeed);
      launcher.index(indexSpeed);
    }
    double launchTriggerValue = RobotContainer.driver.getRawAxis(3);
    if(launchTriggerValue > .02)
    {
      launcher.launch(launchTriggerValue);
    }
    else
    {
      launcher.launch(0);
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLaunching();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!isAuto)
    {
    return false;
    }
    else
    {
      return timer.get() > 3;
    }
  }
}
