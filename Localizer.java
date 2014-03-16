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
    private int counterLarge = 0, counterSmall = 0, distance = 0, tempDistance = 0, intensity = 0, tempIntensity = 0;

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
        nav.turnTo(Math.PI / 2);

        // self-explanator
        robot.rightColor.setFloodlight(true);
        localizeLightly();
        robot.rightColor.setFloodlight(false);

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

        LCD.drawString("LOCATING...", 0, 0);

        double angleX1 = 0, angleX2 = 0, angleY1 = 0, angleY2 = 0;//values of angles seen when gridlines are encountered
        double angleX = 0, angleY = 0; // angleX will be angleX2 - angleX1, same with angleY
        double d = 12; // distannce from the light sensor to the center of rotation, measured with a ruler
        double updatedX = 0, updatedY = 0, deltaTheta = 0; //values to update odometer according to will be assigned to those variables
        
        // start rotating and clock all 4 gridlines
        nav.setMotorRotateSpeed(100); //start rotating
        while (getFilteredColorData() > 50); // hold executing the code until you see a grid line
        nav.stop(); // once grid line is seen, keep executing the code and stop navigating

        angleX1 = Math.toDegrees(odometer.getTheta()); // latch angle at which first grid line is seen to angleX1
        Sound.beep(); //beep to express that angle has been latched
        
        
        nav.setMotorRotateSpeed(100);
        while (getFilteredColorData() < 50); //need this line to keep rotating while still on the grid line
        while (getFilteredColorData() > 50); //now that the grid line is passed, we need this part to keep rotating
        nav.stop(); // again, stop navigating once a grid line is seen

        angleY1 = Math.toDegrees(odometer.getTheta()); //latch second angle to angleY1
        Sound.beep(); //beep to express that angle has been latched
        
        
        //two more of the same block to get angleX2 and angleY2
        nav.setMotorRotateSpeed(100);
        while (getFilteredColorData() < 50); 
        while (getFilteredColorData() > 50);
        nav.stop();

        angleX2 = Math.toDegrees(odometer.getTheta());
        Sound.beep();
        
        nav.setMotorRotateSpeed(100);
        while (getFilteredColorData() < 50);
        while (getFilteredColorData() > 50); 
        nav.stop();

        angleY2 = Math.toDegrees(odometer.getTheta());
        Sound.beep();
        
        angleX = angleX2 - angleX1; //the difference in degrees between the two grid lines faced on x axis
        angleY = angleY2 - angleY1; //the difference in degrees between the two grid lines faced on y axis
        
        // do trig to compute (0,0) and 0 degrees
        updatedX = -d * Math.cos(Math.toRadians(angleY/2)); // formulas from the tutorial to calculate current position
        updatedY = -d * Math.cos(Math.toRadians(angleX/2)); // with respect to the given origin
        deltaTheta = Math.toRadians(270 - angleY2 + (angleY/2));
        
        //update odometer values according to the calculations
        odometer.setPosition(new double [] {updatedX, updatedY, deltaTheta + odometer.getTheta()}, new boolean [] {true, true, true});
        
        // when done travel to (0,0) and turn to 0 degrees
        nav.travelTo(0,0);
        nav.turnTo(0);
   
        // reset localizing flag
        robot.localizing = false;
    }

    /**
     * Correct coordinates based on starting corner information provided
     */
    public void correct() {
        switch(corner) {
            case 1: // (0, 0) tiles
                break;
            case 2: // (10, 0) tiles
                odometer.setX(odometer.getX() + 10 * 30.48);
                break;
            case 3: // (10, 10) tiles
                odometer.setX(odometer.getX() + 10 * 30.48);
                odometer.setY(odometer.getY() + 10 * 30.48);
                break;
            case 4: // (0, 10) tiles
                odometer.setY(odometer.getY() + 10 * 30.48);
                break;
        }
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
            counterLarge = 0;
        } else if (distance < 50 && ++counterSmall > 15) {
            this.distance = distance;
            counterSmall = 0;
            counterLarge = 0;
        }

        this.tempDistance = distance;
        
        return this.distance;
    }

    /**
     * Returns filtered data from the color sensor nearest to
     * the direction the robot is turning
     * 
     * @param direction         direction robot is turning, 1 for CW and 0 for CCW
     *
     * @return the light intensity
     */
    private int getFilteredColorData() {
        int intensity;
        
        // TODO: make use of both color sensors

        // register intensity
        intensity = robot.rightColor.getLightValue();

        // filter out incorrect values that are over 50 or under 30
        if (intensity >= 50 && ++counterLarge > 15) {
            this.intensity = intensity;
            counterSmall = 0;
            counterLarge = 0;
        } else if (intensity < 50 && ++counterSmall > 15) {
            this.intensity = intensity;
            counterSmall = 0;
            counterLarge = 0;
        }

        this.tempIntensity = intensity;
        
        return this.intensity;
    }
    
}
