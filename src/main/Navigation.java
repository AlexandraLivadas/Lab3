package main;
import java.text.DecimalFormat;
import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import odometer.Odometer;
import ultrasonic.UltrasonicPoller;

public class Navigation extends Thread{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;


	private final double WHEEL_RAD = 2.2;
	private final double WHEEL_BASE = 10.0;
	private final double TILE_SIZE;

	

	private Odometer odometer;
	private double x, y, theta;

	private ArrayList<double[]> _coordsList;

	private boolean isNavigating;

	public Navigation(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double tileSize) {

		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.TILE_SIZE = tileSize;
		this.isNavigating = false;
		this._coordsList = new ArrayList<double[]>();

	}

	/**
	 * Has the robot move to a given position
	 * 
	 * @param navX coordinate of position
	 * @param navY coordinate of position
	 */
	void travelTo(double navX, double navY) {
		this._coordsList.add(new double[] {navX*TILE_SIZE, navY*TILE_SIZE});
	}

	public void run() {
		while (!this._coordsList.isEmpty()) {
			double[] coords = this._coordsList.remove(0);
			this._travelTo(coords[0], coords[1]);
		}
	}

	boolean _travelTo(double navX, double navY) {


		UltrasonicPoller usPoller = null;
		if (UltrasonicPoller.getInstance() != null) {
			usPoller = UltrasonicPoller.getInstance();
		}

		// get current coordinates

		//need to convert theta from degrees to radians
		theta = odometer.getXYT()[2];
		x = odometer.getXYT()[0];
		y = odometer.getXYT()[1];	

		// find angle to turn to
		double deltaX = navX - x;
		double deltaY = navY - y;

		// get absolute values of deltas
		double absDeltaX = Math.abs(deltaX);
		double absDeltaY = Math.abs(deltaY);

		double deltaTheta = Math.atan2(deltaX, deltaY) / Math.PI * 180;

		// turn to the correct direction

		this._turnTo(theta, deltaTheta);
		Sound.beep();
		// move until destination is reached
		// while loop is used in case of collision override

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			double newTheta, newX, newY;

			//need to convert theta from degrees to radians
			newTheta = odometer.getXYT()[2];
			newX = odometer.getXYT()[0];
			newY = odometer.getXYT()[1];	

			if (Math.pow(newX - x, 2) + Math.pow(newY - y, 2) > Math.pow(absDeltaX, 2) + Math.pow(absDeltaY, 2)) {
				break;
			}

			// thread override
			if (usPoller != null) {
				if (usPoller.isInitializing) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					usPoller.init(); //hopefully blocking
					try {
						synchronized(usPoller.doneAvoiding) {
							while(usPoller.isAvoiding) {
								usPoller.doneAvoiding.wait();
							}
						}
					} catch(InterruptedException e) {
						e.printStackTrace();
					}
					this._coordsList.add(0, new double[] {navX, navY});
					return false;
				}
			}

			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
		leftMotor.stop(true);
		rightMotor.stop(false);
		this.isNavigating = false;
		Sound.twoBeeps();
		return true;
	}

	//	This method causes the robot to turn (on point) to the absolute heading theta. This method
	//	should turn a MINIMAL angle to its target.
	/**
	 * Turns the robot to the absolute (minima) heading theta
	 * 
	 * @param currTheta current theta
	 * @param destTheta to turn robot by
	 */
	void _turnTo(double currTheta, double destTheta) {
		// get theta difference
		double deltaTheta = destTheta - currTheta;
		// normalize theta
		deltaTheta = normalizeAngle(deltaTheta);

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, deltaTheta), false);
	}

	double normalizeAngle(double theta) {
		if (theta <= -180) {
			theta += 360;
		}
		else if (theta > 180) {
			theta -= 360;
		}
		return theta;
	}


	boolean isNavigating() {
		return isNavigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
