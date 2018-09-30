package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Avoidance implements UltrasonicController{

	private static final int FORWARD_SPEED = 250;
	private int ROTATE_SPEED = 150;
	private static final double WHEEL_RAD = 2.2;
	private double WHEEL_BASE = 10.0;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private boolean isNavigating;
	private double theta;
	private double x;
	private double y;
	private double deltaX;
	private double deltaY;
	private double deltaT;
	private double thetaHead;
	private int distance;
	private final int MIN_DISTANCE = 10;
	
	
	

	public Avoidance(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.isNavigating = false;
	}



	public void travelTo(double navX, double navY) {
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
				distance = (int) Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaX, 2));
				
				turnTo(thetaHead);
				
				
				
				leftMotor.setSpeed(FORWARD_SPEED);
				leftMotor.setSpeed(FORWARD_SPEED);
				leftMotor.forward();
				leftMotor.forward();
				
				isNavigating = true;
				
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
				leftMotor.rotate(convertDistance(WHEEL_RAD, distance), false);
				
				isNavigating = false;
		
	}


	private void turnTo(double thetaHead) {
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
	
	public boolean isNavigating() {
		return isNavigating();
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
		return (int)((180.0 *distance)/(Math.PI * radius));
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



	@Override
	public void processUSData(int distance) {
		if (distance < MIN_DISTANCE) {
			leftMotor.stop();
			rightMotor.stop();
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, 90), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, 90), false);
			
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
			
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, 90), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, 90), false);
		}
	}



	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
