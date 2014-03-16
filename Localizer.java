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
    private Navigation nav;

    private final boolean LEFT = false, RIGHT = true;

    private double firstAngle, secondAngle, deltaTheta;
    private int counterLarge = 0, counterSmall = 0, distance = 0, tempDistance = 0;

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
    public void localize() {
        // set localizing flag
        robot.localizing = true;

        // set low speed to improve accuracy
        nav.setMotorRotateSpeed(100);

        LCD.drawString("LOCATING...", 0, 0);
        
        // rotate the robot until it sees no wall
        while (getFilteredData(RIGHT) < 50) {
            robot.leftMotor.forward();
            robot.rightMotor.backward();
        }

        // play sound
        Sound.playTone(3000,100);

        // keep rotating until the robot sees a wall, then latch the angle
        while (getFilteredData(RIGHT) > 30);
        firstAngle = Math.toDegrees(odometer.getTheta());

        // play lower frequency sound
        Sound.playTone(2000,100);

        // switch direction and wait until it sees no wall
        while (getFilteredData(LEFT) < 50) {
            robot.leftMotor.backward();
            robot.rightMotor.forward();
        }

        // play higher frequency sound
        Sound.playTone(3000,100);

        // keep rotating until the robot sees a wall, then latch the angle
        while (getFilteredData(LEFT) > 30);
        secondAngle = Math.toDegrees(odometer.getTheta());

        // play lower frequency sound
        Sound.playTone(2000,100);

        // measure orientation
        deltaTheta = (secondAngle + firstAngle) / 2 - 50;

        // update the odometer position
        odometer.setTheta(Math.toRadians(secondAngle - deltaTheta));

        // reset localizing flag
        robot.localizing = false;

        // turn to 90 degrees
        nav.turnTo(Math.PI / 2);
        
	}

    private int getFilteredData(boolean right) {
        int distance;

        // select correct ultrasonic sensor
        UltrasonicSensor sonic = right ? robot.rightSonic : robot.leftSonic;
        
        // do a ping
        sonic.ping();
        
        // wait for the ping to complete
        try { Thread.sleep(50); } catch (InterruptedException e) {}
        
        // there will be a delay here
        distance = sonic.getDistance();

        // filter out incorrect values that are over 50 or under 30
        if (distance >= 50 && ++counterLarge > 15) {
            this.distance = distance;
            counterSmall = 0;
        } else if (distance < 50 && ++counterSmall > 15) {
            this.distance = distance;
            counterLarge = 0;
        }

        this.tempDistance = distance;
        
        return this.distance;
    }
    
}
