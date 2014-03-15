import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;

public class LightLocalizer {
	private Odometer odo;
	private TwoWheeledRobot robot;
	private ColorSensor ls;
	private Navigation navigation;
	
	public LightLocalizer(Odometer odo, ColorSensor ls, Navigation navigation) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		this.navigation = navigation;
		
		// turn on the light
		ls.setFloodlight(true);
	}
	
	public void doLocalization() {
		double angleX1 = 0, angleX2 = 0, angleY1 = 0, angleY2 = 0;//values of angles seen when gridlines are encountered
		double angleX = 0, angleY = 0; // angleX will be angleX2 - angleX1, same with angleY
		double d = 12; // distannce from the light sensor to the center of rotation, measured with a ruler
		double updatedX = 0, updatedY = 0, deltaTheta = 0; //values to update odometer according to will be assigned to those variables
			
		
		navigation.travelTo(5,5);// drive to location listed in tutorial
		
		double turnBy = 360 - odo.getTheta(); //get how far we're off from zero degrees
		navigation.turnTo(turnBy); //turn by the required difference to face north
		
		// start rotating and clock all 4 gridlines
		navigation.setMotorRotateSpeed(100); //start rotating
		while (getFilteredLight() > 50) {    } // hold executing the code until you see a grid line
		navigation.stop(); // once grid line is seen, keep executing the code and stop navigating
		try {     
			Thread.sleep(500);  
		}    
		catch (InterruptedException e) {    }
		angleX1 = odo.getTheta(); // latch angle at which first grid line is seen to angleX1
		Sound.beep(); //beep to express that angle has been latched
		
		
		navigation.setMotorRotateSpeed(100);
		while (getFilteredLight() < 50) {    } //need this line to keep rotating while still on the grid line
		while (getFilteredLight() > 50) {    } //now that the grid line is passed, we need this part to keep rotating
		navigation.stop(); // again, stop navigating once a grid line is seen
		try {     
			Thread.sleep(500);  
		}    
		catch (InterruptedException e) {    }
		angleY1 = odo.getTheta(); //latch second angle to angleY1
		Sound.beep(); //beep to express that angle has been latched
		
		
		//two more of the same block to get angleX2 and angleY2
		navigation.setMotorRotateSpeed(100);
		while (getFilteredLight() < 50) {    } 
		while (getFilteredLight() > 50) {    } 
		navigation.stop();
		try {     
			Thread.sleep(500);  
		}    
		catch (InterruptedException e) {    }
		angleX2 = odo.getTheta();
		Sound.beep();
		
		navigation.setMotorRotateSpeed(100);
		while (getFilteredLight() < 50) {    } 
		while (getFilteredLight() > 50) {    } 
		navigation.stop();
		try {     
			Thread.sleep(500);  
		}    
		catch (InterruptedException e) {    }
		angleY2 = odo.getTheta();
		Sound.beep();
		
		angleX = angleX2 - angleX1; //the difference in degrees between the two grid lines faced on x axis
		angleY = angleY2 - angleY1;	//the difference in degrees between the two grid lines faced on y axis
		
		// do trig to compute (0,0) and 0 degrees
		updatedX = -d * Math.cos(Math.toRadians(angleY/2)); // formulas from the tutorial to calculate current position
		updatedY = -d * Math.cos(Math.toRadians(angleX/2)); // with respect to the given origin
		deltaTheta = 270 - angleY2 + (angleY/2);
		
		//update odometer values according to the calculations
		odo.setPosition(new double [] {updatedX, updatedY, (deltaTheta + odo.getTheta())}, new boolean [] {true, true, true});
		
		// when done travel to (0,0) and turn to 0 degrees
		navigation.travelTo(0,0);
		turnBy = 360 - odo.getTheta();
		navigation.turnTo(turnBy);
		
	}
	
	// gets 6 light values in an array, sorts them and returns 3rd one. filtered data provided
	public int getFilteredLight() {
		int[] lightValues = new int[6];

		for (int i = 0; i < 6; i++) {
			lightValues[i] = ls.getLightValue();

			try {
				Thread.sleep(10);
			}
			catch (InterruptedException e) {
			}
		}
		
		sort(lightValues);
		int lightValue = lightValues[3];
		
		return lightValue;
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
