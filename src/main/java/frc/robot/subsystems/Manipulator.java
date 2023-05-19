package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Manipulator implements Subsystem {
  CANSparkMax intakeMotor;
  CANSparkMax outputMotor;
  CANSparkMax sidewaysMotor;

  public Manipulator() {
    intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    outputMotor = new CANSparkMax(Constants.OUTPUT_MOTOR_CAN_ID, MotorType.kBrushless);
    sidewaysMotor = new CANSparkMax(Constants.SIDEWAYS_MOTOR_CAN_ID, MotorType.kBrushless);
  }

  public void intake() {
    intakeMotor.set(Constants.INTAKE_SPEED);
  }

  public void moveRight() {
    sidewaysMotor.set(Constants.TRANSLATE_SPEED);
  }

  public void moveLeft() {
    sidewaysMotor.set(-Constants.TRANSLATE_SPEED);
  }

  public void place() {
    intakeMotor.set(-Constants.INTAKE_SPEED);
    outputMotor.set(Constants.INTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(0);
    outputMotor.set(0);
    sidewaysMotor.set(0);
  }
}
