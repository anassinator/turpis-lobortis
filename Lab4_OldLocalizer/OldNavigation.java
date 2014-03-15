import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Motor;


public class Navigation {
	// put your navigation code here 

	private Odometer odometer;
	private TwoWheeledRobot robot;

/*	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 40; */
	
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 50;

	public static final double LEFT_RADIUS = 2.15;
	public static final double RIGHT_RADIUS = 2.15;
	//public static final double WIDTH = 15.1;
	public static final double WIDTH = 15.1;
	
	NXTRegulatedMotor leftMotor = Motor.A;
	NXTRegulatedMotor rightMotor = Motor.B;

	public static boolean isTurning = false;

	private double position [] = new double [3]; 

	public Navigation(Odometer odometer) {
		this.odometer = odometer;
		this.robot = odometer.getTwoWheeledRobot();
	}

	public void travelTo(double x, double y) {
		// USE THE FUNCTIONS setForwardSpeed and setRotationalSpeed from TwoWheeledRobot!




		//updates x and y acorrding to the current position
		odometer.getPosition(position);

		x -= position[0];   
		y -= position[1];


		// updates degree to be turned by according to the current theta read on the odometer
		double turnBy = Math.toDegrees(Math.atan2(x, y));
		turnBy	-= (position[2]);

		// proceeds to turn
		turnTo(turnBy); 


		//robot.setForwardSpeed(FORWARD_SPEED);
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED); 

		//calculates the hipotenus and goes through it
		double d = Math.sqrt(Math.pow(x,2)+Math.pow(y,2));
		//robot.rotateWheelsDistance(d); 
		leftMotor.rotate(convertDistance(LEFT_RADIUS,d), true);
		rightMotor.rotate(convertDistance(RIGHT_RADIUS,d), false); 

	}

	// gets minimal angle to be turned. subtracts 360 if larger than 180, adds 360 if smaller than -180
	public static double getMinimalAngle(double degree) {    
		if (degree > 180)  
			return getMinimalAngle(degree - 360);		    
		else if (degree < -180)   
			return getMinimalAngle(degree + 360);
		return degree;
	} 


	/*public boolean isNavigating() {
		return isNavigating;
	} */

	public void setMotorSpeeds(double leftSpeed, double rightSpeed) {  
		leftMotor.setSpeed((int) leftSpeed); 
		rightMotor.setSpeed((int) rightSpeed); 
		if (leftSpeed > 0) {  
			leftMotor.forward(); 
		}   
		else {    
			leftMotor.backward();  
		}   
		if (rightSpeed > 0) {   
			rightMotor.forward(); 
		}   
		else {   
			rightMotor.backward();  
		} 
	} 
	public void setMotorRotateSpeed(double rotateSpeed) {   
		setMotorSpeeds(rotateSpeed, -rotateSpeed); 
	}



	public void turnTo(double theta) {
		// USE THE FUNCTIONS setForwardSpeed and setRotationalSpeed from TwoWheeledRobot!
		isTurning = true;

		//theta is updated to its minimal equivalent
		theta = getMinimalAngle(theta);
		robot.setRotationSpeed(ROTATE_SPEED); 
		robot.rotateWheelsAngle(theta);  

		isTurning = false;
	}


	// added by us, stops both motors simultaneously. saves a line, makes code look cleaner
	public void stop() {   
		leftMotor.stop(true);  
		rightMotor.stop(false); 
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}


