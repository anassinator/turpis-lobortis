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

    private final int CCW = 0, CW = 1;

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
     * Localizes using ultrasonic sensor followed by downward facing
     * color sensors
     */
    public void localize() {
        // self-explanatory
        localizeSonicly();

        // face 90 degrees
        nav.turnTo(Math.PI / 2);

        // travel to new origin
        nav.travelTo(15, 15);

        // self-explanator
        localizeLightly();

        // correct coordinate
        correct();
    }

    /**
     * Localizes using falling edge method with outermost ultrasonic sensor
     * and corrects odometer
     */
    public void localizeSonicly() {
        // set localizing flag
        robot.localizing = true;

        // set low speed to improve accuracy
        nav.setMotorRotateSpeed(100);

        LCD.drawString("LOCATING...", 0, 0);
        
        // rotate the robot until it sees no wall
        while (getFilteredSonicData(CW) < 50) {
            robot.leftMotor.forward();
            robot.rightMotor.backward();
        }

        // play sound
        Sound.playTone(3000,100);

        // keep rotating until the robot sees a wall, then latch the angle
        while (getFilteredSonicData(CW) > 30);
        firstAngle = Math.toDegrees(odometer.getTheta());

        // play lower frequency sound
        Sound.playTone(2000,100);

        // switch direction and wait until it sees no wall
        while (getFilteredSonicData(CCW) < 50) {
            robot.leftMotor.backward();
            robot.rightMotor.forward();
        }

        // play higher frequency sound
        Sound.playTone(3000,100);

        // keep rotating until the robot sees a wall, then latch the angle
        while (getFilteredSonicData(CCW) > 30);
        secondAngle = Math.toDegrees(odometer.getTheta());

        // play lower frequency sound
        Sound.playTone(2000,100);

        // measure orientation
        deltaTheta = (secondAngle + firstAngle) / 2 - 50;

        // update the odometer position
        odometer.setTheta(Math.toRadians(secondAngle - deltaTheta));

        // reset localizing flag
        robot.localizing = false;
	}


    /**
     * Localizes using downward facing color sensors and corrects odometer
     */
    public void localizeLightly() {
        // set localizing flag
        robot.localizing = true;

        // set low speed to improve accuracy
        nav.setMotorRotateSpeed(100);

        // ...

        LCD.drawString("LOCATING...", 0, 0);
   
        // reset localizing flag
        robot.localizing = false;
    }

    /**
     * Correct coordinates based on starting corner information provided
     */
    public void correct() {

    }

    /**
     * Returns filtered data from the ultrasonic sensor nearest to
     * the direction the robot is turning
     * 
     * @param direction         direction robot is turning, 1 for CW and 0 for CCW
     *
     * @return the distance in centimeters
     */
    private int getFilteredSonicData(int direction) {
        int distance;

        // select correct ultrasonic sensor
        UltrasonicSensor sonic = direction == 1 ? robot.rightSonic : robot.leftSonic;
        
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
