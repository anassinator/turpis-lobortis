/**
 * Navigation.java
 * Navigates through the game
 * @author Anass Al-Wohoush, Mohamed Kleit
 * @version 0.2
 */

import lejos.nxt.*;

public class Navigation extends Thread {
    public static Robot robot; 
    public static Odometer odometer;
    public static Map map;
    public static int[] courseInfo; 

    /**
     * Navigation constructor
     *
     * @param robot         robot object containing robot's dimensions and motor ports
     * @param odometer      odometer object containing robot's position and orientation
     * @param map           map object containing all obstacles
     * @param courseInfo    information containing opponent's flag location and drop off zone
     */
    public Navigation(Robot robot, Odometer odometer, Map map, int[] courseInfo) {
        this.robot = robot;
        this.odometer = odometer;
        this.map = map;
        this.courseInfo= courseInfo;
    }

    /**
     * Navigates to opponent's flag, captures it and drops it
     * off while avoiding obstacles
     */
    public void run() {
        // ...
    }

    /** 
     * Travels to coordinates x and y
     *
     * @param x             x coordinate in centimeters
     * @param y             y coordinate in centimeters
     */
    public void travelTo(double x, double y) {
        // ...
    }
    
    /**
     * Computes the minimal angle the robot should turn by
     * according to the current theta read on the odometer
     *
     * @param degree        degree in degrees
     *
     * @return the minimal angle in degrees
     */
    public static double getMinimalAngle(double degree) {    
        if (degree > 180)
			return getMinimalAngle(degree - 360);
		else if (degree < -180)
			return getMinimalAngle(degree + 360);
		return degree;
    } 

    /** 
     * Sets motor speeds
     *
     * @param leftSpeed     speed of left motor in degrees/second
     * @param rightSpeed    speed of right motor in degrees/second
     */
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
    
    /**
     * Sets rotate speed
     *
     * @param rotateSpeed   speed of motor in degrees/second
     */
    public void setMotorRotateSpeed(double rotateSpeed) {   
        setMotorSpeeds(rotateSpeed, -rotateSpeed);
    }

    /** 
     * Turns to the angle computed in getMinimalAngle method
     *
     * @param theta         orientation in degrees
     */
    public void turnTo(double theta) {
        // USE THE FUNCTIONS setForwardSpeed and setRotationalSpeed from TwoWheeledRobot!
		isTurning = true;
        
		//theta is updated to its minimal equivalent
		theta = getMinimalAngle(theta);
		robot.setRotationSpeed(ROTATE_SPEED);
		robot.rotateWheelsAngle(theta);
        
		isTurning = false;
    }

    /**
     * Stops the robot
     */
    public void stop() {   
        leftMotor.stop(true);
		rightMotor.stop(false);
    }

    /**
     * Avoids obstacle
     */
    public void avoid() {
        // ...
    }
    
    /**
     * Computes the angle each wheel should rotate 
     * in order to travel a certain distance
     *
     * @param radius        radius in centimeters
     * @param distance      distance in centimeters
     *
     * @return angle (in degrees) each wheel should rotate
     */
    private static int convertDistance(double radius, double distance) {
        // ...
        return 0;
    }
}
