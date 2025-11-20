// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  private final TalonFX m_lManipulator = new TalonFX(6);
  private final TalonFX m_rManipulator = new TalonFX(5); 
  
  public Command mEject() {
    return run(()-> {
      m_lManipulator.setVoltage(3);
      m_rManipulator.setVoltage(3);
    });
  }
  public Command mIntake(){ 
    return run(()-> {
      m_lManipulator.setVoltage(-3);
      m_rManipulator.setVoltage(-3);
    }); 
  }
  /** Creates a new Manipulator. */
  public Manipulator() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
