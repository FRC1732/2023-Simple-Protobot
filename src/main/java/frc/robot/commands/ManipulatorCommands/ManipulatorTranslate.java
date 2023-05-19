package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class ManipulatorTranslate extends CommandBase {
  private Manipulator manipulator;
  private boolean direction;

  public ManipulatorTranslate(Manipulator manipulator, boolean direction) {
    this.manipulator = manipulator;
    this.direction = direction;
    addRequirements(this.manipulator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (this.direction) this.manipulator.moveLeft();
    else this.manipulator.moveRight();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
