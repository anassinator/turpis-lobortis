import java.io.IOException;
import lejos.nxt.*;
import lejos.nxt.comm.*;
import lejos.nxt.remote.*;
import lejos.util.*;

/**
 * Contains robot's properties
 * @author  Anass Al-Wohoush, Mohamed Kleit
 * @version 0.7
 */
public class Robot {
    public final double leftRadius = 2.13;                      // RADIUS OF LEFT WHEEL
    public final double rightRadius = 2.16;                     // RADIUS OF RIGHT WHEEL
    public final double width = 16.20;                          // DISTANCE BETWEEN WHEELS
    public final double distanceBetweenColorSensors = 14.40;    // DISTANCE BETWEEN DOWN-FACING COLOR SENSORS
    public final double distanceToUltrasonicSensors = 14.25;    // DISTANCE FROM CENTER TO ULTRASONIC SENSORS

    // MOTORS
    public NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B, clawMotor = Motor.C;

    // ULTRASONIC SENSORS
    public static UltrasonicSensor leftSonic;
    public static UltrasonicSensor centerSonic;
    public static UltrasonicSensor rightSonic;

    // COLOR SENSORS
    public static ColorSensor leftColor = new ColorSensor(SensorPort.S1, 7);
    public static ColorSensor centerColor = new ColorSensor(SensorPort.S2, 7);
    public static ColorSensor rightColor = new ColorSensor(SensorPort.S3, 7);

    // STORE WHETHER TRYING TO LOCALIZE OR NOT
    public boolean localizing = false;

    // SLAVE NXT
    public RemoteNXT slave = null;

    // CLAW
    public Claw claw = new Claw(clawMotor);

    /**
     * Robot constructor
     */
    public Robot() {
        // MOVE ARM UP
        // claw.grab();

        // CONNECT TO SLAVE
        try {
            LCD.clear();
            LCD.drawString("Connecting...",0,0);
            slave = new RemoteNXT("TEAM14-2", RS485.getConnector());
            LCD.clear();
            LCD.drawString("Connected",0,1);
            Sound.systemSound(false, 1);
            Delay.msDelay(500);
        } catch (IOException e) {
            LCD.clear();
            LCD.drawString("Failed",0,0);
            Sound.systemSound(false, 4);
            Delay.msDelay(2000);
            System.exit(1);
        }
        LCD.clear();

        // SET UP ULTRASONIC SENSORS CONNECTED TO SLAVE
        leftSonic = new UltrasonicSensor(slave.S1);
        centerSonic = new UltrasonicSensor(slave.S2);
        rightSonic = new UltrasonicSensor(slave.S3);

        leftSonic.continuous();
        centerSonic.continuous();
        rightSonic.continuous();
    }
}
