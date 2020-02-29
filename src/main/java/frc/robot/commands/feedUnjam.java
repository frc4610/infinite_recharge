/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class feedUnjam extends CommandBase {
  private Launcher launcher;
  private Intake intake;
  /**
   * Creates a new feedUnjam.
   */
  public feedUnjam(Launcher tLauncher, Intake tIntake) {
    launcher = tLauncher;
    intake = tIntake;
    addRequirements(tLauncher);
    addRequirements(tIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operatorAButton.get())
    {
    launcher.feed(-Constants.feedNEOSpeed);
    launcher.index(-.4);
    }
    intake.intakeCells(-.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.feed(0);
    launcher.stopLaunching();
    intake.intakeCells(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!RobotContainer.operatorAButton.get() && !RobotContainer.operatorBButton.get());
  }
}
