// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.AlignToCargo;
import frc.robot.commands.PIDDriveFoward;
import frc.robot.Limelight;
//import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SeekCargo extends SequentialCommandGroup {
  /** Creates a new SeekCargo. */
  private Limelight LI;
  public SeekCargo() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(new AlignToCargo(RobotContainer.getDrivetrain()),new CargoDriveFoward(RobotContainer.getDrivetrain(), LI.getDistance(0)) );
  }
}
