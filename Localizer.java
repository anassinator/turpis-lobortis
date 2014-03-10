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

    /**
     * Localizer constructor
     * <p>
     * @param robot object containing all color sensor ports
     * @param odometer object containing x, y and theta coordinates
     * @param map object containing surrounding objects
     * @param corner where robot started
     */
    public Localizer(Robot robot, Odometer odometer, Map map, int corner) {
        // ...
        this.robot = robot;
        this.odometer = odometer;
        this.map = map;
        this.corner = corner;
    }
    
    /**
     * Localizes using ultrasonic sensor and color sensor and corrects odometer
     */
    public void localize() {
        // ...
    }
}
