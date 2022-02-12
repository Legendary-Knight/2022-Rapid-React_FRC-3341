// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Limelight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AlignToCargo;

/** An example command that uses an example subsystem. */
public class CargoDriveFoward extends CommandBase {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Drivetrain driveTrain;
    private double errorX,x,kp,distance,error,speed,offset=0;
    private Limelight LI;
    //private int direction;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public CargoDriveFoward(Drivetrain dt,double  dis) {
    driveTrain=dt;
    // Use addRequirements
    dis=distance;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    LI= RobotContainer.getLI();
    LI.changePipeline(Limelight.pipelines.Cargo);
    x=LI.getX();
    errorX=x-offset;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x=LI.getX();
    errorX=x-offset;
    error = distance-driveTrain.getEncoderDistance();
    speed = error/Math.abs(distance);
    if(Math.abs(speed) > .9){
      speed=.9 *  (Math.abs(speed)/speed);
    }
    else if(Math.abs(speed)< .1){
      speed=.1 * (Math.abs(speed)/speed);
    }
    driveTrain.tankDrive(speed, speed);

    if(Math.abs(errorX)<=2){
      AlignToCargo ATC = new AlignToCargo(RobotContainer.getDrivetrain());
    }
    /*
    x=LI.getX();
    error=x-offset;
    speed = error*kp;

    if(Math.abs(speed) > .9){
      speed=.9 *  (Math.abs(speed)/speed);
    }
    else if(Math.abs(speed)< .1){
      speed=.1 * (Math.abs(speed)/speed);
    }
    driveTrain.tankDrive(speed, -speed);
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    errorX=x-offset;
    double acceptance =1;
    if(Math.abs(error)<=acceptance || LI.){
      return true;
    }
    return false;
  }
}
