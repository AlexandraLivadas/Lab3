package ultrasonic;

import lejos.hardware.motor.*;

public class UltrasonicBangBang {


	private final int bandCenter = 30;
	private final int bandwidth = 3;
	private final int motorLow = 150;
	private final int motorHigh = 250;

	private int distance;
	private int filterControl;
	private static final int FILTER_OUT = 80;

	private final double WHEEL_RAD;
	private final double WHEEL_BASE;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private EV3LargeRegulatedMotor usMotor;

	public UltrasonicBangBang(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor usMotor,
			final double WHEEL_RAD, final double WHEEL_BASE) {
		// Default Constructor

		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;

		this.WHEEL_BASE = WHEEL_BASE;
		this.WHEEL_RAD = WHEEL_RAD;

		this.filterControl = 0;

		//This section of code was removed because it caused the motors to start running before the sensor was activated, leading to error
		//    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		//    WallFollowingLab.rightMotor.setSpeed(motorHigh);
		//    WallFollowingLab.leftMotor.forward();
		//    WallFollowingLab.rightMotor.forward();
	}


	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
		if (distance >= 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} else if (distance >= 255) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		} else {
			// distance went below 255: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)

		int error = this.distance - bandCenter;

		//If the error is within the limits, continue forward
		if (Math.abs(error) <= bandwidth) {
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}
		//If the error is negative, move farther from the wall (right turn)
		else if (error < 0) {
			//An even more negative error means that there is a convex corner, requiring a bigger adjustment
			if (error < -this.bandwidth) {
				leftMotor.setSpeed(motorLow);
				rightMotor.setSpeed(motorLow);

				leftMotor.forward();
				rightMotor.backward();
			}
			else {
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorLow);

				leftMotor.forward();
				rightMotor.forward();
			}
		}

		//A positive error means we need to move closer to the wal
		else if (error > 0) {
			if (error < 10 + bandwidth) {
				leftMotor.setSpeed(motorLow);
				rightMotor.setSpeed(motorHigh);

				leftMotor.forward();
				rightMotor.forward();
			} else {
				UltrasonicPoller usPoller = UltrasonicPoller.getInstance();
				try {
					usMotor.rotate(45);
					usPoller.isAvoiding = false;
				} finally {
					synchronized(usPoller.doneAvoiding) {
						usPoller.doneAvoiding.notify();
					}
				}		
			}
		}
	}

	public void initBangBang() {

		usMotor.setSpeed(50);
		usMotor.rotate(-45, false);

		leftMotor.setSpeed(motorLow);
		rightMotor.setSpeed(motorLow);

		leftMotor.rotate(convertAngle(WHEEL_RAD, WHEEL_BASE, 45), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, WHEEL_BASE, 45), false);

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

	public int readUSDistance() {
		return this.distance;
	}
}