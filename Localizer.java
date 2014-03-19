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

    private final int CCW = 0, CW = 1, LEFT = 0, RIGHT = 1;

    private double firstAngle, secondAngle, deltaTheta;
    private int counterLarge = 0, counterSmall = 0, distance = 0, tempDistance = 0, intensity = 0;

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
        nav.turnTo(Math.PI / 4);

        // self-explanator
        localizeLightly();

        nav.travelTo(0, 0);
        nav.turnTo(Math.PI / 2);

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

        // set color sensor floodlight
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);

        // store coordinates
        double[][] posLeft = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};
        double[][] posRight = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0}};

        // detect all four lines
        int counterLeft = 0, counterRight = 0, timerLeft = 0, timerRight = 0;
        boolean done = false;
        while (!done) {
            nav.setMotorSpeeds(100, 100);
            if (--timerLeft <= 0 && isLine(LEFT)){
                odometer.getPosition(posLeft[counterLeft++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                timerLeft = 500;
            }
            if (--timerRight <= 0 && isLine(RIGHT)) {
                odometer.getPosition(posRight[counterRight++], new boolean[] { true, true, true });
                Sound.playTone(2000,100);
                timerRight = 500;
            }

            if (counterLeft == 2 && counterRight == 2)
                done = true;
        }

        // stop
        nav.stop();

        // calculate
        double deltaLeft = distance(posLeft[0], posLeft[1]);
        double deltaRight = distance(posRight[0], posRight[1]);

        double thetaLeft = (posLeft[0][2] + posLeft[1][2])/2;
        double thetaRight = (posRight[0][2] + posRight[1][2])/2;

        double deltaX = -deltaLeft * Math.cos(thetaLeft);
        double deltaY = -deltaRight * Math.cos(thetaRight);

        // reset color sensor floodlight
        robot.leftColor.setFloodlight(false);
        robot.rightColor.setFloodlight(false);

        // correct position
        odometer.setX(odometer.getX() + deltaX);
        odometer.setY(odometer.getY() + deltaY);
   
        // reset localizing flag
        robot.localizing = false;
    }


    /**
     * Returns distance between two sets of coordinates passed in an array
     * 
     * @param firstPosition         first array of X and Y coordinates
     * @param secondPosition        second array of X and Y coordinates
     *
     * @return the distance in centimeters
     */
    public double distance(double[] firstPosition, double[] secondPosition) {
        return Math.sqrt(Math.pow(secondPosition[0] - firstPosition[0], 2) + Math.pow(secondPosition[1] - firstPosition[1], 2));
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
        UltrasonicSensor sonic = (direction == CW) ? robot.rightSonic : robot.leftSonic;
        
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
    private boolean isLine(int side) {        
        // register intensity
        ColorSensor color = (side == RIGHT) ? robot.rightColor : robot.leftColor;
        int intensity = color.getNormalizedLightValue();

        LCD.drawString(String.valueOf(intensity), 0, 3 + side);

        // filter out incorrect values that are over 50 or under 30
        if (intensity <= 500)
            return true;
        else
            return false;
    }
    
}
