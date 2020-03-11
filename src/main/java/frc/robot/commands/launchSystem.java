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
  private Timer timer;
  private boolean isAuto;
  /**
   * Creates a new launchSystem.
   */
  public launchSystem(Launcher tLauncher, double IndexSpeed, boolean auto) {
    launcher = tLauncher;
    indexSpeed = IndexSpeed;
    timer = new Timer();
    isAuto = auto;
    addRequirements(tLauncher);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double launchTriggerValue = RobotContainer.driver.getRawAxis(3);
    if((launcher.GetLauncherSpeed() + 175) >= Constants.launchMaxVelocity*SmartDashboard.getNumber("Manual Launch Power", .5))
      {
        launcher.feed(Constants.feedNEOSpeed);
      }
    else 
      {
        launcher.feed(0);
      }
    if(timer.get() > Constants.feedDelay)
    {
      launcher.index(indexSpeed);
    }
    if(launchTriggerValue > .02)
    {
      launcher.launch(SmartDashboard.getNumber("Manual Launch Power", .5));
    }
    else if(isAuto)
    {
      launcher.launch(.48);
    }
    else
    {
      launcher.stopLaunching(true);
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.stopLaunching(true);
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
      return timer.get() > 4;
    }
  }
}
