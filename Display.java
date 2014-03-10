/**
 * Display.java
 * Displays robot's position and orientation  
 * @author: Anass AL-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Display extends Thread {
	private Odometer odometer;
	private double [] pos;
    private static final long DISPLAY_PERIOD = 20;
	
	/**
	 * Display constructor
	 * @param odometer object containing robot's position and orientation
	 */
	public Display(Odometer odometer) {
		this.odometer = odometer;
	}
	
	/**
	 * Peridically updates odometer data on the screen
	 */
	public void run() {
        long displayStart, displayEnd;
        double[] position = new double[3];

        while (true) {
            displayStart = System.currentTimeMillis();

            print();
            
            // throttle the OdometryDisplay
            displayEnd = System.currentTimeMillis();
            if (displayEnd - displayStart < DISPLAY_PERIOD) {
                try {
                    Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
                } catch (InterruptedException e) {
                    // there is nothing to be done here because it is not
                    // expected that OdometryDisplay will be interrupted
                    // by another thread
                }
            }
        }
	}

	/**
	 * Prints the robot's x and y coordinates in centimeters
	 * and orientation in degrees
	 */
	public void print() { 
		odometer.getPosition(pos, new boolean[] { true, true, true });
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString(String.valueOf(Math.round(pos[0] * 100.0) / 100.0), 3, 0);
		LCD.drawString(String.valueOf(Math.round(pos[1] * 100.0) / 100.0), 3, 1);
		LCD.drawString(String.valueOf(Math.round(pos[2] * 100.0) / 100.0), 3, 2);
	}
}	
	