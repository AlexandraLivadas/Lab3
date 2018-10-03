package main;
import java.text.DecimalFormat;
import lejos.hardware.lcd.TextLCD;
import odometer.Odometer;
import odometer.OdometerExceptions;
import ultrasonic.UltrasonicPoller;

/**
 * This class is used to display the content of the odometer variables (x, y, Theta)
 */
public class Display extends Thread implements Runnable {

  private Odometer odo;
  private UltrasonicPoller usPoller;
  private TextLCD lcd;
  private double[] position;
  private final long DISPLAY_PERIOD = 25;
  private long timeout = Long.MAX_VALUE;

  /**
   * This is the class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    usPoller = UltrasonicPoller.getInstance();
    this.lcd = lcd;
  }

  /**
   * This is the overloaded class constructor
   * 
   * @param odoData
   * @throws OdometerExceptions 
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer();
    this.timeout = timeout;
    this.lcd = lcd;
  }

  public void run() {
    
    lcd.clear();
    
    long updateStart, updateEnd;

    long tStart = System.currentTimeMillis();
    do {
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();
     
      
      
      
      // Print x,y, and theta information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
      lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
      lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      
      
      if (usPoller != null) {
          lcd.drawString("Distance: " + numberFormat.format(usPoller.distance), 0, 3);
    	  
      }
      
//      // DEBUG
//      int[] tachos = odo.getTachoCount();
//      lcd.drawString("TL: " + numberFormat.format(tachos[0]), 0, 3);
//      lcd.drawString("TR: " + numberFormat.format(tachos[1]), 0, 4);
      
//      try {
//		if (OdometryCorrection.getInstance() != null) {
//			  OdometryCorrection OC = OdometryCorrection.getInstance();
//			  	lcd.drawString("LS: " + numberFormat.format(OC.lightSensorIntensity), 0, 5);
//			  	lcd.drawString("Std dev: " + numberFormat.format(OC.stdDev), 0, 6);
//			  	lcd.drawString("Mean: " + numberFormat.format(OC.mean), 0, 7);
//		  }
//	  } catch (OdometerExceptions e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//	  }
      
      
      // this ensures that the data is updated only once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    } while ((updateEnd - tStart) <= timeout);

  }

}
