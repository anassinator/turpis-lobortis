/**
 * Correction.java
 * Corrects odometry based on line readings from downward facing
 * color sensors
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */

import lejos.nxt.*;

public class Correction extends Thread {
    private Robot robot;
    private Odometer odometer;

    private double firstAngle, secondAngle, deltaTheta;
    private final int LEFT = 0, RIGHT = 1;

    /**
     * Correction constructor
     * 
     * @param robot             robot object containing all color sensor ports
     * @param odometer          odometer object containing x, y and theta coordinates
     */
    public Correction(Robot robot, Odometer odometer) {
        // ...
        this.robot = robot;
        this.odometer = odometer;

        // set color sensor floodlight
        robot.leftColor.setFloodlight(true);
        robot.rightColor.setFloodlight(true);
    }
    
    /**
     * Corrects odometer using downward facing color sensors based on angle of attack
     */
    public void run() {
        while (true) {
            // store coordinates
            double[] posLeft = {0.0, 0.0, 0.0};
            double[] posRight = {0.0, 0.0, 0.0};

            // detect both lines
            int timerLeft = 0, timerRight = 0, counter = 0, lastDone = 0;
            boolean done = false;
            while (!done) {
                if (--timerLeft <= 0 && isLine(LEFT)){
                    odometer.getPosition(posLeft, new boolean[] { true, true, true });
                    Sound.playTone(2000,100);
                    timerLeft = 500;
                    counter++;
                    lastDone = LEFT;
                }
                if (--timerRight <= 0 && isLine(RIGHT)) {
                    odometer.getPosition(posRight, new boolean[] { true, true, true });
                    Sound.playTone(2000,100);
                    timerRight = 500;
                    counter++;
                    lastDone = RIGHT;
                }

                if (counter == 2)
                    done = true;
            }


            // calculate
            double distance = distance(posLeft, posRight);

            double deltaTheta = Math.asin(robot.distanceBetweenColorSensors / distance);

            if (lastDone == LEFT)
                deltaTheta = -deltaTheta;

            // correct position
            odometer.setTheta(odometer.getTheta() + deltaTheta);
        }
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
     * Returns whether a line is under the selected color sensor is above a
     * a black line
     * 
     * @param side         select color sensor, 1 for RIGHT and 0 for LEFT
     *
     * @return <code>true</code> if line detected; <code>false</code> otherwise
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
