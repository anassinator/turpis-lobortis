/**
 * Localizer.java
 * Localizes robot at beginning of run based on sensor readings
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Localizer {
    private static Robot robot;
    private static Odometer odometer;
    private static Map map;
    private static int corner;
    private static Navigation nav;

    /**
     * Localizer constructor
     * 
     * @param robot             robot object containing all color sensor ports
     * @param odometer          odometer object containing x, y and theta coordinates
     * @param map               map object containing surrounding objects
     * @param corner            corner where robot started
     */
    public Localizer(Robot robot, Odometer odometer, Map map, Navigation nav, int corner) {
        // ...
        this.robot = robot;
        this.odometer = odometer;
        this.map = map;
        this.nav = nav;
        this.corner = corner;
    }
    
    /**
     * Localizes using falling edge method with central ultrasonic sensor
     * and corrects using downward facing color sensors and corrects odometer
     */
    public static void localize() {
        robot.localizing = true;

        double angleA = 0, angleB = 0;
		double deltaTheta;
			
		nav.setMotorRotateSpeed(100); //start rotating

		while (getFilteredData() < 90) {    }  // keep rotating and don't execute code as long as you are looking at the wall
        
		while (23 < getFilteredData()) {    } //  keep rotating and don't execute code as long as you are facing away from the wall
		
		angleA = Math.toDegrees(odometer.getTheta()); // latch angleA once you see the wall
		Sound.beep();  // make a beep sound to demonstrate latching
		
		nav.setMotorRotateSpeed(-100); // and start rotating again in opposite direction
		while (getFilteredData() < 90) {    } // keep rotating and don't execute code as long as you are looking at the wall
		nav.stop(); //take a break once you stop seeing the wall

		
		nav.setMotorRotateSpeed(-100); // start rotating again in the opposite direction
		while (23 < getFilteredData()) {    } // keep rotating until you see the wall and don't execute code
		nav.stop(); //once you see the wall stop rotating
		angleB = Math.toDegrees(odometer.getTheta()); // latch angleB
		Sound.beep(); // make a beep sound to demonstrate latching
		
		//formula taken from the tutorial file
		deltaTheta = Math.toRadians((angleA + angleB)/2 + 10);
        
		//update the odometer information according to the deltaTheta found from calculations
		odometer.setPosition(new double [] {0.0, 0.0, deltaTheta + odometer.getTheta()}, new boolean [] {false, false, true});

        robot.localizing = false;
		
		//set robot to 0 degrees facing north
		nav.turnTo(Math.PI / 2);  
        
	}
    
    
    // gets 6 distance values in an array, sorts them and returns 3rd one. filtered data provided
	public static int getFilteredData() {
		int[] distances = new int[6];
        
		// do a ping
		for (int i = 0; i < 6; i++) {
			robot.centerSonic.ping();
            
			// there will be a delay here
			distances[i] = robot.centerSonic.getDistance();
		}
		sort(distances);
		int distance = distances[3];
		if (distance > 100) {
			distance = 100;
		}
        
		return distance;
	}

	private static void sort(int[] array) {
		int length = array.length;
		for (int i = 0; i < length; i++) {
			for (int j = 1; j < length - i; j++) {
				if (array[j - 1] > array[j]) {
					int t = array[j - 1];
					array[j - 1] = array[j];
					array[j] = t;
				}
			}
		}
	}
    
}
