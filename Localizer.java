/**
 * Localizer.java
 * Localizes robot at beginning of run based on sensor readings
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Localizer {

    private Robot robot;
    private Odometer odometer;
    private Map map;
    private int corner;
    private Navi

    /**
     * Localizer constructor
     * 
     * @param robot             robot object containing all color sensor ports
     * @param odometer          odometer object containing x, y and theta coordinates
     * @param map               map object containing surrounding objects
     * @param corner            corner where robot started
     */
    public Localizer(Robot robot, Odometer odometer, Map map, int corner) {
        // ...
        this.robot = robot;
        this.odometer = odometer;
        this.map = map;
        this.corner = corner;
    }
    
    /**
     * Localizes using ultrasonic sensor and color sensor and corrects odometer
     */
    public void localize() {
        double angleA = 0, angleB = 0;
		double deltaTheta;
        
		if (locType == LocalizationType.FALLING_EDGE) {
			
			nav.setMotorRotateSpeed(100); //start rotating
			while (getFilteredData() < 90) {    }  // keep rotating and don't execute code as long as you are looking at the wall
            
			nav.setMotorRotateSpeed(100);  // keep rotating
			while (23 < getFilteredData()) {    } //  keep rotating and don't execute code as long as you are facing away from the wall
			
			angleA = odometer.getTheta(); // latch angleA once you see the wall
			Sound.beep();  // make a beep sound to demonstrate latching
			
			nav.setMotorRotateSpeed(-100); // and start rotating again in opposite direction
			while (getFilteredData() < 90) {    } // keep rotating and don't execute code as long as you are looking at the wall
			nav.stop(); //take a break once you stop seeing the wall
			try {
				Thread.sleep(500);
			}
			catch (InterruptedException e) {    }
            
			
			nav.setMotorRotateSpeed(-100); // start rotating again in the opposite direction
			while (23 < getFilteredData()) {    } // keep rotating until you see the wall and don't execute code
			nav.stop(); //once you see the wall stop rotating
			angleB = odometer.getTheta(); // latch angleB
			Sound.beep(); // make a beep sound to demonstrate latching
			
			//formula taken from the tutorial file
			if ( angleA > angleB)
				deltaTheta = 45 - ((angleA + angleB)/2);
			else
				deltaTheta = 225 - ((angleA + angleB)/2);
			
            
			//update the odometer information according to the deltaTheta found from calculations
			odometer.setPosition(new double [] {0.0, 0.0, (deltaTheta + odometer.getTheta())}, new boolean [] {false, false, true});
			try {
				Thread.sleep(500);
			}
			catch (InterruptedException e) {    }
			
			//set robot to 0 degrees facing north
			double turnBy = 360 - odometer.getTheta();
			nav.turnTo(turnBy);  
            
		}
    }
}
