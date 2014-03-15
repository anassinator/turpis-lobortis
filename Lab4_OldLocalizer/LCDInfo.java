import lejos.nxt.LCD;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class LCDInfo implements TimerListener{
	public static final int LCD_REFRESH = 100;
	private Odometer odo;
	private Timer lcdTimer;
	private USLocalizer usl; //we're going to need this a couple of lines below
	private LightLocalizer ll; // and this as well
		
	// arrays for displaying data
	private double [] pos;
	
	public LCDInfo(Odometer odo, USLocalizer usl, LightLocalizer ll) { // modified to constructor to take USLocalizer and LightLocalizer type arguments
		this.odo = odo;
		this.lcdTimer = new Timer(LCD_REFRESH, this);
		this.usl = usl; //our usl is matched to the usl coming from the main method in Lab4 class
		this.ll = ll; //same here
		
		// initialise the arrays for displaying data
		pos = new double [3];
		
		// start the timer
		lcdTimer.start();
	}
	
	
	public void timedOut() { 
		odo.getPosition(pos);
		int distance = usl.getFilteredData(); //we're getting info on distance from usl object 
		int lightVal = ll.getFilteredLight(); // same here, light value info
		LCD.clear();
		LCD.drawString("X: ", 0, 0);
		LCD.drawString("Y: ", 0, 1);
		LCD.drawString("H: ", 0, 2);
		LCD.drawString("Dis: ", 0, 3);
		LCD.drawString("Li: ", 0, 4);
		LCD.drawString(String.valueOf(Math.round(pos[0] * 100.0) / 100.0), 3, 0); //this prints up to 2 decimals
		LCD.drawString(String.valueOf(Math.round(pos[1] * 100.0) / 100.0), 3, 1);
		LCD.drawString(String.valueOf(Math.round(pos[2] * 100.0) / 100.0), 3, 2);
		LCD.drawInt(distance, 5, 3); //printing distance on LCD screen
		LCD.drawInt(lightVal, 5, 4); //printing light value on LCD screen
		
	}
}
