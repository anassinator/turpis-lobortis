/**
 * Navigation.java
 * @author: Mohamed Kleit, Anass Al-Wohoush
 * @version 0.1
 */

import lejos.nxt.*;

public class Navigation {
	
	public static Robot robot = new Robot; 
	public static Odomoter odometer = new Odometer(robot);
	public static Map map = new Map(odometer);
	public static int[] courseInfo; 
	
	

		public Navigation(Robot robot, Odometer odometer,Map map, int[] courseInfo) {
			
			this.robot = robot;
			this.odometer = odometer;
			this.map = map;
			this.courseInfo= courseInfo;
	}


		/** 
		 * Travels to the point of coordinates x and y
		 * @param x in cm
		 * @param y in cm
		 */
		public void travelTo(double x, double y) {

		}

	
		
		/**
		 * Computes the minimal angle the robot should turn by
		 * according to the current theta read on the odometer
		 * @param degree in degrees
		 * @return the minimal angle in degrees
		 */
		public static double getMinimalAngle(double degree) {    
			
		} 


		
		/** 
		 * Sets motor speeds
		 * @param leftSpeed in deg/sec
		 * @param rightSpeed in deg/sec
		 */
		public void setMotorSpeeds(double leftSpeed, double rightSpeed) {  
			
		} 
		
		
		/**
		 * Sets rotate speed 
		 * @param rotateSpeed
		 */
		public void setMotorRotateSpeed(double rotateSpeed) {   
			setMotorSpeeds(rotateSpeed, -rotateSpeed); 
		}


		/** 
		 * Turns to the angle computed in getMinimalAngle method
		 * @param theta in degrees
		 */
		public void turnTo(double theta) {
	
		}

		

		/**
		 * Stops the robot
		 */
		public void stop() {   
 
		}

		
		/**
		 * Computes the angle each wheel should rotate 
		 * in order to travel a certain distance
		 * @param radius in cm
		 * @param distance in cm
		 * @return angle (in degrees) each wheel should rotate
		 */
		private static int convertDistance(double radius, double distance) {

		}
		
		
	}

	
