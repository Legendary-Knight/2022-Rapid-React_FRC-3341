// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Limelight {
    private NetworkTable table;
	private NetworkTableEntry tv, tx, ty, ta;
	private int pipeline = 0;
	private int v;
	private double x, y, area;
	private int lineC=0, lineH=2;
	//private static double targetArea;
	//private static double ballheight = 3.5; //inches
    //private static double cameraheight = 10.5; //inches
	private double ang1=0, ang2=0,h1=0,h2=0;
    private double distance;
	public enum pipelines{Cargo,Hub};

	private int test=0;
    
    public Limelight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		if(table!=null){
			test=0;
		}
		else{
			test=1;
		}
		SmartDashboard.putNumber("table True",test);
		tv = table.getEntry("tv");
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");

		if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
			lineC=0;
		}
		else{
			lineC=1;
		}

    }
    
    public void update() {
		table.getEntry("pipeline").setNumber(pipeline);
		table.getEntry("ledMode").setNumber(0);
		v = (int) tv.getDouble(0.0);
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);

		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);
		SmartDashboard.putNumber("area", area);
		SmartDashboard.putBoolean("v", v == 1 ? true : false);
    }

    public  void disable() {
		table.getEntry("pipeline").setNumber(0);
	}

	public void setPipeline(int num) {
		if (num >= 0 && num <= 9) {
			pipeline = num;
		}
	}

	public  int getV() {
		return v;
    }
    
	public double getX() {
		return x;
    }
    
	public  double getY() {
		return y;
    }
    
	public double getArea() {
		System.out.println("area:"+area);
		return area;
	}

	public  void flash() {
		table.getEntry("ledMode").setNumber(2);
    }
    
    public double getDistance(double H2){
		h2= H2;
		ang2=y;
        //distance=(43.3*Math.pow(area, -.531));
        //System.out.println("Distance(Inches):"+distance);
        //System.out.println("distance: "+distance);
		distance = (h2-h1)/(Math.tan(ang1+ang2));
        return distance;
	}
	public void changePipeline(pipelines pipeline){
		if(pipeline == pipelines.Cargo){
			setPipeline(lineC);
		}
		else if(pipeline == pipelines.Hub){
			setPipeline(lineH);
		}

	}
	public void switchPipeline(){
		if(pipeline==lineC){
			pipeline=lineH;
		}
		else{
			pipeline=lineC;
		}
	}

	public pipelines Cargo(){
		return pipelines.Cargo;
	}
	public pipelines Hub(){
		return pipelines.Hub;
	}
}