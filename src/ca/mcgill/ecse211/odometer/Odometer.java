package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private static Odometer odo = null; // Returned as singleton
  
  private double x, y, theta;

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  
  private int oldLeftMotorTachoCount;
  private int oldRightMotorTachoCount;
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double WHEEL_BASE;
  private final double WHEEL_RAD;

  private double[] position;

  public static Lock lock = new ReentrantLock(true);
  private volatile boolean isReseting = false;
  private Condition doneReseting = lock.newCondition();

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
    final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odo.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.oldLeftMotorTachoCount = 0;
    this.oldRightMotorTachoCount = 0;
    this.WHEEL_BASE = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }



/**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
 * @throws OdometerExceptions 
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  public int[] getTachoCount() {
	  int[] tachos = new int[2];
	  tachos[0] = leftMotorTachoCount;
	  tachos[1] = rightMotorTachoCount;
	  return tachos;
  }
  
  public double[] calcXYT() {
	  
	  double XYT[] = new double[3];
      oldLeftMotorTachoCount = leftMotorTachoCount;
      oldRightMotorTachoCount = rightMotorTachoCount;
      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();
      
      double distL = Math.PI*WHEEL_RAD*(leftMotorTachoCount-oldLeftMotorTachoCount)/180;  

      double distR = Math.PI*WHEEL_RAD*(rightMotorTachoCount-oldRightMotorTachoCount)/180;   // displacements 

      double deltaD = 0.5*(distL+distR);       // compute vehicle displacement 
      double deltaT = (distL-distR)/WHEEL_BASE;        // compute change in heading 
      theta += deltaT;  // update heading 
      double dX = deltaD * Math.sin(theta);    // compute X component of displacement 
      double dY = deltaD * Math.cos(theta);  // compute Y component of displacement
      XYT[0] = dX;
      XYT[1] = dY;
      XYT[2] = deltaT/Math.PI*180;
      return XYT;
      
  }
  
  
  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    while (true) {
      updateStart = System.currentTimeMillis();



      // TODO Calculate new robot position based on tachometer counts
      double[] xyt = calcXYT();
      
      // TODO Update odometer values with new calculated values
      odo.update(xyt[0], xyt[1], xyt[2]);

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          // there is nothing to be done
        }
      }
    }
  }

//  public void update(double dx, double dy, double dtheta) {
//	    lock.lock();
//	    isReseting = true;
//	    try {
//	      x += dx;
//	      y += dy;
//	      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
//	                                                    // within 360
//	                                                    // degrees
//	      isReseting = false; // Done reseting
//	      doneReseting.signalAll(); // Let the other threads know that you are
//	                                // done reseting
//	    } finally {
//	      lock.unlock();
//	    }
//	    
//  }
//
//public double[] getXYT() {
//    double[] position = new double[3];
//    lock.lock();
//    try {
//      while (isReseting) { // If a reset operation is being executed, wait
//        // until it is over.
//        doneReseting.await(); // Using await() is lighter on the CPU
//        // than simple busy wait.
//      }
//
//      position[0] = x;
//      position[1] = y;
//      position[2] = theta;
//
//    } catch (InterruptedException e) {
//      // Print exception to screen
//      e.printStackTrace();
//    } finally {
//      lock.unlock();
//    }
//
//    return position;
//
//  }
//
//public void setXYT(double x, double y, double theta) {
//    lock.lock();
//    isReseting = true;
//    try {
//      this.x = x;
//      this.y = y;
//      this.theta = theta;
//      isReseting = false; // Done reseting
//      doneReseting.signalAll(); // Let the other threads know that you are
//                                // done reseting
//    } finally {
//      lock.unlock();
//    }
//  }
//
//public void setX(double x) {
//    lock.lock();
//    isReseting = true;
//    try {
//      this.x = x;
//      isReseting = false; // Done reseting
//      doneReseting.signalAll(); // Let the other threads know that you are
//                                // done reseting
//      //Sound.beep();
//    } finally {
//      lock.unlock();
//    }
//  }
//
//public void setY(double y) {
//    lock.lock();
//    isReseting = true;
//    try {
//      this.y = y;
//      isReseting = false; // Done reseting
//      doneReseting.signalAll(); // Let the other threads know that you are
//                                // done reseting
//      //Sound.beep();
//    } finally {
//      lock.unlock();
//    }
//  }
//
//public void setTheta(double theta) {
//    lock.lock();
//    isReseting = true;
//    try {
//      this.theta = theta;
//      isReseting = false; // Done reseting
//      doneReseting.signalAll(); // Let the other threads know that you are
//                                // done reseting
//      //Sound.beep();
//    } finally {
//      lock.unlock();
//    }
//  }
}
