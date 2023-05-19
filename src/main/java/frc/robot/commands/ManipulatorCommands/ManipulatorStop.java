package frc.robot.commands.ManipulatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Manipulator;

public class ManipulatorStop extends CommandBase {
  private Manipulator manipulator;

  public ManipulatorStop(Manipulator manipulator) {
    this.manipulator = manipulator;
    addRequirements(this.manipulator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.manipulator.stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
