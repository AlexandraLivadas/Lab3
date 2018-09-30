package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerDisplay;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
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

public class Lab3 {

  // Motor Objects, and Robot related parameters
  private static final EV3LargeRegulatedMotor leftMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
  private static final EV3LargeRegulatedMotor rightMotor =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
  private static final Port lsPort = LocalEV3.get().getPort("S4"); //light sensor port
  private static final Port usPort = LocalEV3.get().getPort("S1"); //light sensor port
  
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();
  public static final double WHEEL_RAD = 2.2;
  public static final double WHEEL_BASE = 10.0;
  
  public static final SensorModes lsSensor = new EV3ColorSensor(lsPort);
  public static final SampleProvider lsColor = lsSensor.getMode("Red");
  public static final float[] sampleColor = new float[lsColor.sampleSize()];
  
  public static final SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
  public static final SampleProvider usDistance = usSensor.getMode("Distance"); 
  public static final float[] usData = new float[usDistance.sampleSize()];
  
  protected static final double TILE_SIZE = 30.48; 
  public static int currentPoint;
  public static int[] xValues = {0, 1, 2, 2, 1};
  public static int[] yValues = {2, 1, 2, 1, 0};
  
  
  public static int getLightSensor() {
	  lsColor.fetchSample(sampleColor, 0); // acquire data
	  return (int) (sampleColor[0] * 100.0);
  }
   
  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice;
    
    // Odometer related objects
    final Odometer odometer = new Odometer(leftMotor, rightMotor, WHEEL_BASE, WHEEL_RAD);
    OdometerDisplay odometryDisplay = new OdometerDisplay(lcd);
    UltrasonicPoller usPoller = null;
        
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
    } 
    while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

    		Thread odoThread = new Thread(odometer);
    		odoThread.start();
    		Thread odoDisplayThread = new Thread(odometryDisplay);
    		odoDisplayThread.start();
    		
        	currentPoint = 0; 
    	
    if (buttonChoice == Button.ID_LEFT) {
    		
    		//Navigation with no obstacles
    	  (new Thread() {
			public void run() {
    	        //Driver.start(odometer, leftMotor, rightMotor, WHEEL_RAD, WHEEL_BASE, false);
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
    	    }).start();

    } else {
    		(new Thread() {
    			public void run() {
    				//Driver.start(odometer, leftMotor, rightMotor, WHEEL_RAD, WHEEL_BASE, true);
    				Avoidance avoid = new Avoidance(leftMotor, rightMotor, odometer);
    				while(currentPoint<5) {
    					//We need to make sure it isn't already navigating somewhere
    					if(!avoid.isNavigating()) {
    						avoid.travelTo(xValues[currentPoint]*TILE_SIZE, yValues[currentPoint]*TILE_SIZE);
    						currentPoint++;
    					}
    				}
    			}
  	    }).start();
    		usPoller = new UltrasonicPoller(usDistance, usData);
    		usPoller.start();


    }

    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
    System.exit(0);
  }
  

}

