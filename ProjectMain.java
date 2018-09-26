package Project;



import lejos.hardware.Button;
import lejos.hardware.Sound;

import java.util.LinkedList;
import java.util.concurrent.TimeUnit;

import Project.UltrasonicLocalizer.LocType;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @author Team 3
 * 
 * Main class that'll launch the initial threads and contains all constant values
 *
 */
public class ProjectMain {

	static Odometer odometer;
	static Display odometryDisplay;
	static Navigate navigation;
	static UltrasonicPoller uspoller;
	static OdometryCorrection correctOdo;


	public static double odo_x;
	public static double odo_y;
	public static double odo_theta;

	public static int greenCorner;

	public static double block1X;
	public static double block1Y;
	
	public static int greenTeam;


	/* lower left hand corner of the green zone x coordinate */
	public static int Green_LL_X;

	/* lower left hand corner of the green zone y coordinate  */
	public static int Green_LL_Y;

	/* upper right hand corner of the green zone x coordinate  */
	public static int Green_UR_X;

	/* upper right hand corner of the green zone y coordinate */
	public static int Green_UR_Y;

	/* lower left hand corner of the green zone x coordinate */
	public static int Red_LL_X;

	/* lower left hand corner of the green zone y coordinate  */
	public static int Red_LL_Y;

	/* upper right hand corner of the green zone x coordinate  */
	public static int Red_UR_X;

	/* upper right hand corner of the green zone y coordinate */
	public static int Red_UR_Y;

	/* lower left hand corner of the tunnel footprint x coordinate */
	public static int TN_LL_X;

	/* lower left hand corner of the tunnel footprint y coordinate */
	public static int TN_LL_Y;

	/* upper right hand corner of the tunnel footprint x coordinate */
	public static int TN_UR_X;

	/* upper right hand corner of the tunnel footprint y coordinate */
	public static int TN_UR_Y;

	/* lower left hand corner of the bridge footprint x coordinate */
	public static int BR_LL_X;

	/* lower left hand corner of the bridge footprint y coordinate */
	public static int BR_LL_Y;

	/*upper right hand corner of the bridge footprint x coordinate  */
	public static int BR_UR_X;

	/* upper right hand corner of the bridge footprint y coordinate */
	public static int BR_UR_Y;

	/*starting corner x coordinate*/
	public static int ST_X;

	/*starting corner y coordinate*/
	public static int ST_Y;

	/*get ultrasonic sensor readings in cm, *100 multiplier*/
	public static final double usCorrection = 100.0;

	/*Motor speed for USLocalization*/
	public static int LOC_SPEED = 50;

	/*distance between the two forward light sensors*/
	public static double offset = 17.4;

	public static boolean tunnelFirst = true;

	public static double TILE_SIZE = 30.48;

	public static int FORWARD_SPEED = 150;

	public static int ROTATE_SPEED = 75;
	
	public static int navCounter = 0;


	/* Motors */
	static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));


	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 13.5;

	/* us Sensors*/
	private static final Port usPort = LocalEV3.get().getPort("S1");
	static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	static SampleProvider usDistance = usSensor.getMode("Distance");
	public static float[] usData = new float[usDistance.sampleSize()];

	/*light sensors*/

	//color sensor for color search
	//	private static final Port FlagPort = LocalEV3.get().getPort("S4");
	//	public static final EV3ColorSensor flagSensor = new EV3ColorSensor(FlagPort);

	//color sensor for odometer correction
	public static final Port RightSensorPort = LocalEV3.get().getPort("S2");
	public static final EV3ColorSensor RightSensor = new EV3ColorSensor(RightSensorPort);
	public static final SampleProvider rightSensorVal = RightSensor.getRedMode();
	public static final float[] rightSensorValData = new float[RightSensor.sampleSize()];


	//color sensor for localization
	public static final Port LeftSensorPort = LocalEV3.get().getPort("S3");
	public static final EV3ColorSensor LeftSensor = new EV3ColorSensor(LeftSensorPort);
	public static final SampleProvider leftSensorVal = LeftSensor.getRedMode();
	public static float[] leftSensorValData = new float[LeftSensor.sampleSize()];

	/* state machine*/
	static State state;

	/**@authors : Team 3
	 * 
	 * main method that launches the first threads
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		//		state = State.WIFI;
		//		Wifi wifi = new Wifi();
		//		wifi.getValues();
		//		while (state == State.WIFI) {
		//			continue;
		//		}

		final TextLCD lcd = LocalEV3.get().getTextLCD();
		lcd.clear();

		odometer = Odometer.getOdometer(Navigate.leftMotor, Navigate.rightMotor, TRACK, WHEEL_RAD); 
		odometryDisplay = new Display(lcd);
		navigation = new Navigate(odometer);
		uspoller = new UltrasonicPoller(usDistance, usData, navigation);
		correctOdo = new OdometryCorrection(odometer);
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(odometer, LocType.FALLING_EDGE, uspoller);

		TN_LL_X = 2;
		TN_LL_Y = 3;
		TN_UR_X = 3;
		TN_UR_Y = 5;
		BR_LL_X = 5;
		BR_LL_Y = 3;
		BR_UR_X = 6;
		BR_UR_Y = 5;
		Red_LL_X = 0;
		Red_LL_Y = 5;
		Red_UR_X = 6;
		Red_UR_Y = 8;
		Green_LL_X = 1;
		Green_LL_Y = 1;
		Green_UR_X = 3;
		Green_UR_Y = 3;
		ST_X = 7;
		ST_Y = 1;

		if(tunnelFirst) {
			double x1 = TN_LL_X+0.5;
			double y1 = TN_LL_Y-1;
			double x2 = TN_UR_X-0.5;
			double y2 = TN_UR_Y+1;
			double x3 = BR_UR_X-0.5;
			double y3 = BR_UR_Y+1;
			double x4 = BR_LL_X+0.5;
			double y5 = BR_LL_Y-1;
		}

		LinkedList<Integer> coordinates = new LinkedList<Integer>();
		coordinates.addLast(TN_LL_X);
		coordinates.addLast(TN_LL_Y);
		coordinates.addLast(TN_UR_X);
		coordinates.addLast(TN_UR_Y);
		coordinates.addLast(BR_UR_X);
		coordinates.addLast(BR_UR_Y);
		coordinates.addLast(BR_LL_X);
		coordinates.addLast(BR_LL_X);

		//navigation.setPath(coordinates);

		//Localization starts here

		state = State.FIRSTLOCALIZATION;

		lcd.clear();
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread displayThread = new Thread(odometryDisplay);
		displayThread.start();
		Thread nav = new Thread(navigation);
		nav.start();
		

//		Thread usLocThread = new Thread (usLoc);
//		usLocThread.start();
//
//		while (usLoc.locdone && navCounter == 0) {
//			Thread nav = new Thread(navigation);
//			nav.start();
//			navCounter = 1;
//		}



		//		state = State.FIRSTLOCALIZATION;
		//		//light localization
		//		LightLocalizer lightloc = new LightLocalizer (odometer);
		//		lightloc.run();
		//
		//		Button.waitForAnyPress();
		//
		//		//Navigation starts here 
		//		state = State.NAVIGATION;
		//		navigating(navigation);
//		odometer.setXYT(7*30.48, 1*30.48, 270);
//		if (tunnelFirst) {
//			state = State.TUNNEL;
//		}
//		else {
//			state = State.BRIDGE;
//		}
//
//		if (state == State.TUNNEL) {
//			Navigate.navToTunnel();
//			correctOdo.run();
//			Navigate.leftMotor.stop();
//			Navigate.rightMotor.stop();
//			odometer.setXYT((TN_LL_X*30.48)+30.48/2, TN_LL_Y-1, 0);
//			Navigate.tunnelTraversal2();
//			state = State.AFTERTUNNEL;
//		}
//
//		if (state == State.BRIDGE) {
//			Navigate.bridgeTraversal();
//			correctOdo.detectLine();
//			Navigate.turnTo(-90);
//			correctOdo.detectLine();
//			odometer.setXYT((BR_UR_X-1)*30.48, (BR_UR_Y+1)*30.48, 270);
//
//			state = State.AFTERBRIDGE;
//		}
//
//		if (state == State.AFTERTUNNEL) {
//			state = State.RETURNBRIDGE;
//		}
//
//		if (state == State.AFTERBRIDGE) {
//			Navigate.returnBridge();
//			state = State.RETURNBRIDGEB;
//		}
//
//		if (state == State.RETURNBRIDGE) {
//			Navigate.returnBridge();
//		}

		//		while (state != State.NAVIGATION) {
		//
		//			if (state == State.FLAGSEARCH) {
		//				//flagsearch()
		//				state = State.NAVIGATION;
		//			}
		//
		//			if (state ==  State.BRIDGE) {
		//				navigation.bridgeTraversal();
		//				state = State.AFTERBRIDGE;
		//			}
		//
		//			if(state == State.RETURNBRIDGE) {
		//				navigation.returnBridge();
		//				Navigate.travelTo(ST_X, ST_Y);
		//			}
		//
		//			if (state == State.TUNNEL)
		//			{
		//				Lab5.leftMotor.forward();
		//				Lab5.rightMotor.forward();
		//				navigation.tunnelTraversal();
		//				state= State.RETURNBRIDGE;
		//			}
		//
		//			if (state == State.AFTERBRIDGE) {
		//				correctOdo.run();
		//				Navigate.travelTo(BR_UR_X-1,  BR_UR_Y+1);
		//			//	lightloc.run();
		//				state = State.RETURNBRIDGE;
		//			}
		//
		//			if (state == State.AFTERTUNNEL) {
		//			//	lightloc.run();
		//			}
		//
		//		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);

	}
}
