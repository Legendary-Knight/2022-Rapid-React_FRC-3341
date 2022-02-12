// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.AlignToCargo;
import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AlignToHub extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain driveTrain;
    private double x, y, turnS, fowardS, speed, speedR, speedL, errorX, errorD, KX, kD=1, offset=0, distance,targetD, H2=104; //H2  os  Height of the Hub 8 feet 8 inches (104 inches)
    private Limelight LI;
    //private int direction;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlignToHub(Drivetrain dt) {
    driveTrain=dt;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LI= RobotContainer.getLI();
    LI.changePipeline(Limelight.pipelines.Hub);
    x=LI.getX();
    y=LI.getY();
    distance=LI.getDistance(H2);
    errorX=x-offset;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    distance=LI.getDistance(H2);
    errorD = targetD-distance;
    fowardS= errorD*kD;
    
    if(Math.abs(fowardS) > .75){
      fowardS=.9 *  (Math.abs(fowardS)/fowardS);
    }
    else if(Math.abs(speed)< .1){
      fowardS =.1 * (Math.abs(fowardS)/fowardS);
    }


    x=LI.getX();
    errorX=x-offset;
    turnS = errorX*KX;

    if(Math.abs(turnS) > .25){
      speed=.5 *  (Math.abs(turnS)/turnS);
    }
    else if(Math.abs(speed)< .1){
      speed=.1 * (Math.abs(turnS)/turnS);
    }

    speedL = fowardS+turnS;
    speedR = fowardS-turnS;

    if(Math.abs(speedL) > 1){
      speed=.9 *  (Math.abs(speedL)/speedL);
    }
    else if(Math.abs(speedL)< .1){
      speed=.1 * (Math.abs(speedL)/speedL);
    }
    if(Math.abs(speedR) > .9){
      speed=.9 *  (Math.abs(speedR)/speedR);
    }
    else if(Math.abs(speedR)< .1){
      speed=.1 * (Math.abs(speedR)/speedR);
    }

    driveTrain.tankDrive(speedL, speedR);

    /*
    x=LI.getX();
    errorX=x-offset;
    speed = errorX*kp;

    


    driveTrain.tankDrive(speed, -speed);
    */


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    x=LI.getX();
    errorX=x-offset;

    distance=LI.getDistance(H2);
    errorD = targetD-distance;
    double acceptanceX =1;
    double acceptanceD =1;
    if(Math.abs(errorX)<=acceptanceX && Math.abs(errorD)<=acceptanceD){
      return true;
    }
    return false;
  }
}
