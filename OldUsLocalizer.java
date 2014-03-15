
import lejos.nxt.Button;
import lejos.nxt.Sound;
import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 30;

	private Odometer odo;
	private TwoWheeledRobot robot;
	private UltrasonicSensor us;
	private LocalizationType locType;
	private Navigation navigation;
	
	private Object lock; // just added

	public USLocalizer(Odometer odo, UltrasonicSensor us, LocalizationType locType, Navigation navigation) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;
		this.locType = locType;
		this.navigation = navigation;
		// switch off the ultrasonic sensor
		us.off();
	}

	public void doLocalization() {
		double angleA = 0, angleB = 0;
		double deltaTheta;

		if (locType == LocalizationType.FALLING_EDGE) {
			
			navigation.setMotorRotateSpeed(100); //start rotating
			while (getFilteredData() < 90) {    }  // keep rotating and don't execute code as long as you are looking at the wall

			navigation.setMotorRotateSpeed(100);  // keep rotating 
			while (23 < getFilteredData()) {    } //  keep rotating and don't execute code as long as you are facing away from the wall
			
			angleA = odo.getTheta(); // latch angleA once you see the wall
			Sound.beep();  // make a beep sound to demonstrate latching
			
			navigation.setMotorRotateSpeed(-100); // and start rotating again in opposite direction
			while (getFilteredData() < 90) {    } // keep rotating and don't execute code as long as you are looking at the wall
			navigation.stop(); //take a break once you stop seeing the wall
			try {     
				Thread.sleep(500);  
			}    
			catch (InterruptedException e) {    }

			
			navigation.setMotorRotateSpeed(-100); // start rotating again in the opposite direction
			while (23 < getFilteredData()) {    } // keep rotating until you see the wall and don't execute code
			navigation.stop(); //once you see the wall stop rotating
			angleB = odo.getTheta(); // latch angleB
			Sound.beep(); // make a beep sound to demonstrate latching
			
			//formula taken from the tutorial file
			if ( angleA > angleB)
				deltaTheta = 45 - ((angleA + angleB)/2);
			else
				deltaTheta = 225 - ((angleA + angleB)/2);
			
		
			//update the odometer information according to the deltaTheta found from calculations
			odo.setPosition(new double [] {0.0, 0.0, (deltaTheta + odo.getTheta())}, new boolean [] {false, false, true});
			try {     
				Thread.sleep(500);  
			}    
			catch (InterruptedException e) {    }
			
			//set robot to 0 degrees facing north
			double turnBy = 360 - odo.getTheta();
			navigation.turnTo(turnBy);  
		
		} 
		else {
			//RISING EDGE
			//same logic as the falling edge. This time starts facing away from the wall
			navigation.setMotorRotateSpeed(100);
			while (23 < getFilteredData()) {    }
			navigation.stop();
			try {     
				Thread.sleep(500);  
			}    
			catch (InterruptedException e) {    }
			navigation.setMotorRotateSpeed(100);   
			while (getFilteredData() < 90) {    } 
			
			navigation.stop();
			angleA = odo.getTheta();
			Sound.beep(); 
			try {     
				Thread.sleep(500);  
			}    
			catch (InterruptedException e) {    }
			
			
			navigation.setMotorRotateSpeed(-100);
			while (getFilteredData() < 90) {    } 
			navigation.stop(); 
			navigation.setMotorRotateSpeed(-100);
			while (getFilteredData() < 90) {    } 
			navigation.stop(); 
			angleB = odo.getTheta();
			Sound.beep();
			
			if ( angleA > angleB)
				deltaTheta = 45 - ((angleA + angleB)/2);
			else
				deltaTheta = 225 - ((angleA + angleB)/2);
			
			odo.setPosition(new double [] {0.0, 0.0, (deltaTheta + odo.getTheta())}, new boolean [] {false, false, true});
			try {     
				Thread.sleep(500);  
			}    
			catch (InterruptedException e) {    }
			double turnBy = 360 - odo.getTheta();
			navigation.turnTo(turnBy); 	 

		}
	}
	
	// gets 6 distance values in an array, sorts them and returns 3rd one. filtered data provided
	public int getFilteredData() {
		int[] distances = new int[6];

		// do a ping
		for (int i = 0; i < 6; i++) {
			us.ping();

			// wait for the ping to complete
			try {
				Thread.sleep(40);
			}
			catch (InterruptedException e) {
			}

			// there will be a delay here
			distances[i] = us.getDistance();
		}
		sort(distances);
		int distance = distances[3];
		if (distance > 100) {
			distance = 100;
		}

		return distance;
	}
	private void sort(int[] array) {
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
