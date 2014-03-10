/**
 * Display.java
 * Displays robot's position and orientation  
 * @author: Anass AL-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Display {
	private Odometer odometer;
	private double [] pos;
	
	/**
	 * Display constructor
	 * @param odometer object containing robot's position and orientation
	 */
	public Display(Odometer odometer) {
		this.odometer = odometer;
	}
	
	/**
	 * Prints the robot's x and y coordinates in centimeters
	 * and orientation in degrees
	 */
	public void print() { 
		odometer.getPosition(pos);
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString(String.valueOf(Math.round(pos[0] * 100.0) / 100.0), 3, 0);
		LCD.drawString(String.valueOf(Math.round(pos[1] * 100.0) / 100.0), 3, 1);
		LCD.drawString(String.valueOf(Math.round(pos[2] * 100.0) / 100.0), 3, 2);
	}
}	
	