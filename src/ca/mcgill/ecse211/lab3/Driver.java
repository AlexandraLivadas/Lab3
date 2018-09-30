package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Driver {
	
	private static final double TILE_SIZE = 30.48;
	
	public static void start(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			double radius, double width, boolean avoidance) {
		int currentPoint = 0;
		int[] xValues = {0, 1, 2, 2, 1};
		int[] yValues = {2, 1, 2, 1, 0};
		
		try {
			//Have the thread sleep for 2 seconds
			Thread.sleep(2000*3);
		} catch (InterruptedException e) {
			//THere is nothing to be done here
		}
		
		//If avoidance=false, simply navigate to points
		if (avoidance == false) {
			Navigation navigator = new Navigation(leftMotor, rightMotor, odometer);
			//Loop through all points using the currentPoint value
			while(currentPoint<5) {
				//We need to make sure it isn't already navigating somewhere
				if(!navigator.isNavigating()) {
					navigator.travelTo(xValues[currentPoint]*TILE_SIZE, yValues[currentPoint]*TILE_SIZE);
					currentPoint++;
				}
			}
		}
		
		else {
			Avoidance avoid = new Avoidance(leftMotor, rightMotor, odometer);
			while(currentPoint<5) {
				//We need to make sure it isn't already navigating somewhere
				if(!avoid.isNavigating()) {
					avoid.travelTo(xValues[currentPoint]*TILE_SIZE, yValues[currentPoint]*TILE_SIZE);
					currentPoint++;
				}
			}
		}
	}

}
