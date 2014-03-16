/**
 * Navigation.java
 * Navigates through the game
 * @author Anass Al-Wohoush, Mohamed Kleit
 * @version 0.2
 */

import lejos.nxt.*;

public class Navigation extends Thread {
    private static final int ROTATE_SPEED = 50;
    public static Robot robot; 
    public static Odometer odometer;
    public static Map map;
    public static int[] courseInfo;
    public static boolean turning = false;

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
        // calculate desired location from current location
        // by measuring the difference
        double xToTravel = x - odometer.getX();
        double yToTravel = y - odometer.getY();
    
        // measure desired orientation by trigonometry
        // atan2 deals with correct signs for us
        double desiredOrientation = Math.atan2(yToTravel, xToTravel);

        // spin to desired angle
        turnTo(desiredOrientation);

        // measure desired distance by pythagorus
        double desiredDistance = Math.sqrt(xToTravel * xToTravel + yToTravel * yToTravel);

        // move forward desired distance and return immediately
        robot.leftMotor.rotate(convertDistance(robot.leftRadius, desiredDistance), true);
        robot.rightMotor.rotate(convertDistance(robot.rightRadius, desiredDistance), false);
        
        // stop motors
        stop();    
    }
    
    /**
     * Computes the minimal angle the robot should turn by
     * according to the current theta read on the odometer
     *
     * @param radians      angle in radians
     *
     * @return the minimal angle in radians
     */
    public static double getMinimalAngle(double radians) {    
        if (radians > Math.PI)
			return getMinimalAngle(radians - Math.PI * 2);
		else if (radians < -Math.PI)
			return getMinimalAngle(radians + Math.PI * 2);
		return radians;
    } 

    /** 
     * Sets motor speeds
     *
     * @param leftSpeed     speed of left motor in degrees/second
     * @param rightSpeed    speed of right motor in degrees/second
     */
    public void setMotorSpeeds(double leftSpeed, double rightSpeed) {  
        robot.leftMotor.setSpeed((int) leftSpeed);
		robot.rightMotor.setSpeed((int) rightSpeed);
		if (leftSpeed > 0) {
			robot.leftMotor.forward();
		}
		else {
			robot.leftMotor.backward();
		}
		if (rightSpeed > 0) {
			robot.rightMotor.forward();
		}
		else {
			robot.rightMotor.backward();
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
     * @param theta         orientation in radians
     */
    public void turnTo(double theta) {
       // calculate angle to rotate realtive to current angle
        double currentOrientation = odometer.getTheta();
        double angle = theta - currentOrientation;

        // correct angle to remain within -180 and 180 degrees
        // to minimize angle to spin
        if (angle < -3.14)
            angle += 6.28;
        else if (angle > 3.14)
            angle -= 6.28;

        // set turning flag
        turning = true; 

        // rotate said angle and wait until done
        robot.leftMotor.rotate(-convertAngle(robot.leftRadius, angle), true);
        robot.rightMotor.rotate(convertAngle(robot.rightRadius, angle), false);

        stop();

        // reset turning flag
        turning = false;
    }

    /**
     * Stops the robot
     */
    public void stop() {   
        robot.leftMotor.stop(true);
		robot.rightMotor.stop(false);
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
        return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * Computes the angle each wheel should rotate 
     * in order to rotate in place a certain angle
     *
     * @param radius        radius in centimeters
     * @param distance      distance in centimeters
     *
     * @return angle (in degrees) each wheel should rotate
     */
    private static int convertAngle(double radius, double angle) {
        // fixed to work in radians instead of degrees
        return convertDistance(radius, robot.width * angle / 2.0);
    }
}
