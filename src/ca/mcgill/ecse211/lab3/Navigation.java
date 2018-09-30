package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread{
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	private final double WHEEL_RAD = 2.2;
	private final double WHEEL_BASE = 10.0;
	private Odometer odometer;
	private double deltaX;
	private double deltaY;
	private double deltaT;
	private double x, y, theta;
	private double thetaHead;
	private double distance;
	//private boolean isTurning;
	//private boolean isMoving;
	private boolean isNavigating;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odo) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		isNavigating = false;
	}
	
//	public void run() {
//
//		
//	}
		
/**
 * Has the robot move to a given position
 * 
 * @param navX coordinate of position
 * @param navY coordinate of position
 */
	void travelTo(double navX, double navY) {
		//synchronize to avoid collision
		synchronized(odometer.lock) {
			//need to convert theta from degrees to radians
			theta = odometer.getXYT()[2] * 180/Math.PI;
			x = odometer.getXYT()[0];
			y = odometer.getXYT()[1];
			
		}
		
		this.deltaX = navX - x;
		this.deltaY = navY - y;
		
		//Now, calculate degrees you need to turn from 0 degrees:
		//Math.atan2(y, x) gives angle b/ween x-axis and the vector between (0,0) and (x, y) --> assume (0, 0) is current (x, y) of odometer
		deltaT = Math.atan2(navY - y, navX - x) *180/Math.PI;
		
		//Calculate the actual angle of change
		thetaHead = deltaT - theta;
		
		//Distance it should travel in this direction
		distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaX, 2));
		
		turnTo(thetaHead);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		
		isNavigating = true;
		
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
		
		isNavigating = false;
		}
	


//	This method causes the robot to turn (on point) to the absolute heading theta. This method
//	should turn a MINIMAL angle to its target.
/**
 * Turns the robot to the absolute (minima) heading theta
 * 
 * @param thetaHead to turn robot by
 */
	void turnTo(double thetaHead) {
		theta = odometer.getXYT()[2] * 180/Math.PI;
		
		//Find minimal angle to turn
		if (thetaHead <= -180) {
			thetaHead = thetaHead + 360;
		}
		else if (thetaHead > 180) {
			thetaHead = thetaHead - 360;
		}
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, thetaHead), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, thetaHead), false);
		
		isNavigating = false;
	}
	
	boolean isNavigating() {
		return isNavigating;
	}
	
/**
 * Moves robot a certain distance
 * 
 * @param radius of wheels
 * @param distance 
 * 
 * @return degrees needed to turn in order to move forward that distance
 */
	private static int convertDistance(double radius, double distance) {
		return (int)((180.0 *distance)/(Math.PI * distance));
	}

/**
 * Return degrees to turn robot
 * 
 * @param radius of wheels
 * @param width of wheel base
 * @param angle to turn
 * @return degrees to turn the robot to turn the proper angle amount
 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, (Math.PI*width*angle/360.0));
	}
}

