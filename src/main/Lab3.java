package main;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicBangBang;
import ultrasonic.UltrasonicPoller;

public class Lab3 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S4"); //ultrasonic sensor port
	private static final EV3LargeRegulatedMotor usMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2;
	public static final double WHEEL_BASE = 10.0;
	public static final double TILE_SIZE = 30.48; 
	
	//
	
	public static final double MIN_DISTANCE = 21.0;
	

	//	public static final SensorModes lsSensor = new EV3ColorSensor(lsPort);
	//	public static final SampleProvider lsColor = lsSensor.getMode("Red");
	//	public static final float[] sampleColor = new float[lsColor.sampleSize()];

	public static final SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
	public static final SampleProvider usDistance = usSensor.getMode("Distance"); 
	public static final float[] usData = new float[usDistance.sampleSize()]; 


	public static void main(String[] args) throws OdometerExceptions {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, WHEEL_BASE, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change
		Navigation navigator = new Navigation(odometer,leftMotor, rightMotor, TILE_SIZE);
		//Avoidance avoid = new Avoid(usSensor, odometer, navigator);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Simple | with  ", 0, 2);
			lcd.drawString("navig. | avoid-   ", 0, 3);
			lcd.drawString("       | ance ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {

			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();

			navigator.travelTo(0, 1);
			navigator.travelTo(1, 1);
			navigator.travelTo(2, 0);
			navigator.travelTo(2, 1);
			navigator.travelTo(1, 0);

			Thread navigatorThread = new Thread(navigator);
			navigatorThread.start();

			// Display changes in position as wheels are (manually) moved

			//      Thread odoThread = new Thread(odometer);
			//      odoThread.start();
			//      Thread odoDisplayThread = new Thread(odometryDisplay);
			//      odoDisplayThread.start();

		} else {
			LCD.clear();
			
			UltrasonicBangBang usCont = new UltrasonicBangBang(leftMotor, rightMotor, usMotor, WHEEL_RAD, WHEEL_BASE);
			UltrasonicPoller usPoller = UltrasonicPoller.getInstance(usSensor, usData, usCont, MIN_DISTANCE);
			
			Thread odoThread = new Thread(odometer);
			odoThread.start();

			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			
			Thread ultrasonicThread = new Thread(usPoller);
			ultrasonicThread.start();
			
			navigator.travelTo(0, 2);
			navigator.travelTo(1, 1);
			navigator.travelTo(2, 0);
			navigator.travelTo(2, 1);
			navigator.travelTo(1, 0);

			Thread navigatorThread = new Thread(navigator);
			navigatorThread.start();
			

			//start the avoid thread
			//avoid.start();
			//start the odometer thread
//			Thread odoThread = new Thread(odometer);
//			odoThread.start();
//			//start the odometer display thread
//			Thread odoDisplayThread = new Thread(odometryDisplay);
//			odoDisplayThread.start();

			//      // Start odometer and display threads
			//      Thread odoThread = new Thread(odometer);
			//      odoThread.start();
			//      Thread odoDisplayThread = new Thread(odometryDisplay);
			//      odoDisplayThread.start();
			//      
			//      // spawn a new Thread to avoid SquareDriver.drive() from blocking
			//      (new Thread() {
			//        public void run() {
			//          SquareDriver.drive(leftMotor, rightMotor, WHEEL_RAD, WHEEL_RAD, TRACK);
			//        }
			//      }).start();

			//      // Start correction if right button was pressed
			//      if (buttonChoice == Button.ID_RIGHT) {
			//        Thread odoCorrectionThread = new Thread(odometryCorrection);
			//        odoCorrectionThread.start();
			//        
			//      }


		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}


}
