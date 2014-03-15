import lejos.nxt.*;

public class Lab4 {

	public static void main(String[] args) {
		//int buttonChoice;

		// setup the odometer, display, and ultrasonic and light sensors
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		Navigation navig = new Navigation(odo);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2);
		ColorSensor ls = new ColorSensor(SensorPort.S1); //changed from LightSensor to ColorSensor

		USLocalizer usl = new USLocalizer(odo, us, USLocalizer.LocalizationType.FALLING_EDGE, navig);	
		LightLocalizer ll = new LightLocalizer(odo, ls, navig);
		
		LCDInfo lcd = new LCDInfo(odo, usl, ll); //LCDInfo is initiated here, because we added usl and ll to arguments,
		//which is why we need to initiate them first. The reason we added them as arguments is because we want to
		//print distance light values on LCD
		
		usl.doLocalization(); //perform the ultrasonic localization
		ll.doLocalization(); //perform the light sensor localization

		Button.waitForAnyPress();
	}

}
