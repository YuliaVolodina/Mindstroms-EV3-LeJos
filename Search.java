package Project;

import java.util.ArrayList;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * @author Team 3
 * 
 * Class that'll search for the blocks when the robot is in the search area
 *
 *@implements UltrasonicController, Runnable
 */
public class Search implements UltrasonicController, Runnable {

	public int searchDigit;

	private static final double TILE_SIZE = 30.48;
	public static final int FORWARD_SPEED = 150;
	public static final int ROTATE_SPEED = 75;
	public static final int SWEEP_SPEED = 60;
	public static final double wheel_radius = ProjectMain.WHEEL_RAD;
	double width = ProjectMain.TRACK;

	//Coodinate values
	public static int LLX = 0;
	public static int LLY = 0;
	public static int URX = 2;
	public static int URY = 2;
	public static int dimensionX = URX-LLX;
	public static int dimensionY = URY-LLY;
	
	public ArrayList<Double> xPos = new ArrayList<Double>();
	public ArrayList<Double> yPos = new ArrayList<Double>();
	public ArrayList<Double> distX = new ArrayList<Double>();
	public ArrayList<Double> distY = new ArrayList<Double>();

	private double odo_x, odo_y, odo_theta;

	private int distance;

	public Odometer odometer;

	private static final Port usPortTag = LocalEV3.get().getPort("S1");
	@SuppressWarnings("resource") // Because we don't bother to close this resource
	SensorModes usSensor2 = new EV3UltrasonicSensor(usPortTag); // usSensor is the instance
	SampleProvider usDistance2 = usSensor2.getMode("Distance");
	float[] usData2 = new float[usDistance2.sampleSize()];

	public Search (Odometer odometer) {
		this.odometer = odometer;
	}

	/**
	 * @author Kartik Misra
	 * 
	 * Travel to the search zone and initiate the search
	 * 
	 * @throws InterruptedException
	 */
	public void doSearch() throws InterruptedException {
		if (!((odometer.getXYT()[0] == LLX) && (odometer.getXYT()[1] == LLY))) {
			travelTo(LLX, LLY);
			turnTo(-45);
		}
		sweep();
	}

	/**@author : Kartik Misra
	 * 
	 * drives on the edge of the search area while scanning for block using the second US sensor using the Tag thread
	 * 
	 * @param
	 * @void
	 * @throws InterruptedException
	 */
	public void sweep() throws InterruptedException {
//		Tag tag1 = new Tag (dimensionX, dimensionY, Tag.SearchType.UPSEARCH, odometer);
//		UltrasonicPoller tagPoller1 = new UltrasonicPoller(usDistance2, usData2, tag1);
//		Thread tagThread1 = new Thread (tag1);
//		Thread usPoller1 = new Thread (tagPoller1);
//		tagThread1.start();
//		usPoller1.start();
//		travelTo(LLX, URY);
//		tagThread1.interrupt();
//		usPoller1.interrupt();
		Navigate.turnTo(-90+odometer.getTheta());
		Navigate.sensorMotor.rotate(90);
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, ProjectMain.Green_LL_X-ProjectMain.Green_UR_X),true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, ProjectMain.Green_LL_X-ProjectMain.Green_UR_X),true);
		ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
		this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
		while (Navigate.leftMotor.isMoving() || Navigate.rightMotor.isMoving()) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			if (this.distance < ProjectMain.Green_UR_Y-ProjectMain.Green_LL_Y) {
				double thisY = odometer.getXYT()[1];
				double distanceX = this.distance;
				if(yPos == null && distX == null) {
					yPos.add(new Double(thisY));
					distX.add(new Double(distanceX));
				}
				this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
				if (this.distance < ProjectMain.Green_UR_Y-ProjectMain.Green_LL_Y)
				while (this.distance >= thisY-5 && this.distance <= thisY+5) {
					this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
					if (this.distance < ProjectMain.Green_UR_Y-ProjectMain.Green_LL_Y)
					System.out.println("        "+ this.distance);
				}
			}
		}
		
		turnTo(90);
		
		Navigate.leftMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, ProjectMain.Green_LL_Y-ProjectMain.Green_UR_Y),true);
		Navigate.rightMotor.rotate(Navigate.convertDistance(ProjectMain.WHEEL_RAD, ProjectMain.Green_LL_Y-ProjectMain.Green_UR_Y),true);
		ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
		this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
		while (Navigate.leftMotor.isMoving() || Navigate.rightMotor.isMoving()) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			if (this.distance < ProjectMain.Green_UR_X-ProjectMain.Green_LL_X) {
				double thisY = odometer.getXYT()[0];
				double distanceX = this.distance;
				if(yPos == null && distX == null) {
					yPos.add(new Double(thisY));
					distX.add(new Double(distanceX));
				}
				this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
				if (this.distance < ProjectMain.Green_UR_X-ProjectMain.Green_LL_X)
				while (this.distance >= thisY-5 && this.distance <= thisY+5) {
					this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
					if (this.distance < ProjectMain.Green_UR_X-ProjectMain.Green_LL_X)
					System.out.println("        "+ this.distance);
				}
			}
		}

//		Tag tag2 = new Tag (dimensionX, dimensionY, Tag.SearchType.RIGHTSEARCH, odometer);
//		UltrasonicPoller tagPoller2 = new UltrasonicPoller(usDistance2, usData2, tag2);
//		Thread tagThread2 = new Thread(tag2);
//		Thread usPoller2 = new Thread (tagPoller2);
//		tagThread2.start();
//		usPoller2.start();
//		travelTo(URX,URY);
//		tagThread2.interrupt();
//		usPoller2.interrupt();
	}
	
	public void identify() throws OdometerExceptions {
		ColorID theID = new ColorID(odometer);
		Thread idThread = new Thread (theID);
		idThread.start();
		travelToInCm(ProjectMain.block1X, ProjectMain.block1Y);
		travelTo(URX,URY);
	}
	
	/**
	 * @author: Kartik Misra, Alexis Franche
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction 
	 * 
	 * @param x and y values to travel to
	 * @void commands the robot to rotate and set the travel distance to reach destination
	 * 
	 */
	public void travelToInCm(double pointX, double pointY){
		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];


		odo_x = actualX;
		odo_y = actualY;
		odo_theta = acutalTheta;

		double finalX = pointX;
		double finalY = pointY;


		double yDiff = finalY-odo_y;
		double xDiff = finalX-odo_x;

		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));

		double distFinal = Math.hypot(xDiff,yDiff);

		double angleFinal = angleCalc - odo_theta;

		if(angleFinal < -180){ 
			turnTo(angleFinal + 360);
		}
		else if(angleFinal > 180){
			turnTo(angleFinal - 360);
		}
		else{
			turnTo(angleFinal);
		}
		goTo(distFinal);
	}

	/**
	 * @author: Kartik Misra, Alexis Franche
	 * 
	 * Calculates the distance and angle to reach the input coordinates, converts them in cm and commands the robot to move to 
	 * that direction 
	 * 
	 * @param x and y values to travel to
	 * @void commands the robot to rotate and set the travel distance to reach destination
	 * 
	 */
	public void travelTo(double pointX, double pointY){
		double[] location = new double[3];
		location = odometer.getXYT();
		double acutalTheta = location[2];
		double actualX = location[0];
		double actualY = location[1];


		odo_x = actualX;
		odo_y = actualY;
		odo_theta = acutalTheta;

		double finalX = pointX * TILE_SIZE;
		double finalY = pointY * TILE_SIZE;


		double yDiff = finalY-odo_y;
		double xDiff = finalX-odo_x;

		double angleCalc = Math.toDegrees(Math.atan2(xDiff,yDiff));

		double distFinal = Math.hypot(xDiff,yDiff);

		double angleFinal = angleCalc - odo_theta;

		if(angleFinal < -180){ 
			turnTo(angleFinal + 360);
		}
		else if(angleFinal > 180){
			turnTo(angleFinal - 360);
		}
		else{
			turnTo(angleFinal);
		}
		goTo(distFinal);
	}

	/**
	 * @authors: Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to rotate to the input distance
	 * 
	 * @param angle to turn to
	 * @void command the robot wheels to move to reach input distance
	 */
	public void turnTo(double theta){
		Navigate.leftMotor.setSpeed(ROTATE_SPEED);
		Navigate.rightMotor.setSpeed(ROTATE_SPEED);

		Navigate.leftMotor.rotate(convertAngle(wheel_radius, width, theta), true);
		Navigate.rightMotor.rotate(-convertAngle(wheel_radius, width, theta), false);
	}

	/**
	 * @authors: Alexis Franche, Kartik Misra
	 * 
	 * Commands the robot to move exactly the input diatance
	 * 
	 * @param length of distance to travel when called given the odometer XYT and the final coordinates
	 * @void commands the robot to travel the input distance
	 * 
	 */
	public void goTo(double distance){
		Navigate.leftMotor.setSpeed(FORWARD_SPEED);
		Navigate.rightMotor.setSpeed(FORWARD_SPEED);

		Navigate.leftMotor.rotate(convertDistance(wheel_radius, distance), true);
		Navigate.rightMotor.rotate(convertDistance(wheel_radius, distance), false);
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public void run() {
		try {
			doSearch();
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}
		try {
			identify();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
