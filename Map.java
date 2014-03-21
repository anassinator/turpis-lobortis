import lejos.nxt.*;
import java.util.*;

/**
 * Maps surrounding area using an array of ultrasonic sensors
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.1
 */
public class Map extends Thread {
    private Robot robot;
    private Odometer odometer;

    private ArrayList<Obstacle> obstacles = new ArrayList<Obstacle>();

    /**
     * Map constructor
     *
     * @param robot         robot object containing robot's ultrasonic sensor ports
     * @param odometer      odometer object containing robot's position and orientation
     */
    public Map(Robot robot, Odometer odometer) {
        this.robot = robot;
        this.odometer = odometer;
    }

    /**
     * Runs mapping thread and keeps track of obstacles
     */
    public void run() {
        // ...
    }
}