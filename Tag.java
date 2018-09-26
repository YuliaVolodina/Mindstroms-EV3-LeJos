package Project;

import java.util.ArrayList;

/**
 * @author Kartik Misra
 * 
 * This class runs alongside searching and is in charge of detected blocks and storing they coordinates
 * 
 * @implements UltrasonicController, Runnable
 */

public class Tag implements UltrasonicController, Runnable {
	private int distance;

	public Odometer odometer;

	private int dimX;
	private int dimY;

	public ArrayList<Double> xPos = new ArrayList<Double>();
	public ArrayList<Double> yPos = new ArrayList<Double>();
	public ArrayList<Double> distX = new ArrayList<Double>();
	public ArrayList<Double> distY = new ArrayList<Double>();

	private double block1X, block1Y, block2X, block2Y, block3X, block3Y, block4X, block4Y;

	public enum SearchType {UPSEARCH, RIGHTSEARCH};
	public SearchType searchType;

	public Tag (int dimX, int dimY, SearchType searchType, Odometer odometer) {
		this.odometer = odometer;
		this.dimX = dimX;
		this.dimY = dimY;
		this.searchType = searchType; 
	}

	@Override
	public void run() {
		try {
			tagging();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	/**
	 * @author: Kartik Misra
	 * 
	 * tag blocks and record their positions and store them in the arraylists
	 * 
	 * @void
	 * @throws InterruptedException
	 */
	public void tagging() throws InterruptedException {
		if (this.searchType == SearchType.UPSEARCH) {
			ProjectMain.usDistance.fetchSample(ProjectMain.usData, 0);
			this.distance = (int) (ProjectMain.usData[0]*ProjectMain.usCorrection);
			if (this.distance < 60) {
				double thisY = odometer.getXYT()[1];
				double distanceX = this.distance;
				if (yPos == null && distX == null) {
					yPos.add(new Double(thisY));
					distX.add(new Double(distanceX));
				}
				while (this.distance >= thisY-5 && this.distance <= thisY+5) {
					System.out.println("        "+ this.distance);
				}
			}
		}

		if (this.searchType == SearchType.RIGHTSEARCH) {
			if (this.distance < 60) {
				double thisX = odometer.getXYT()[0];
				double distanceY = this.distance;
				if (xPos == null && distY == null) {
					xPos.add(new Double (thisX));
					distY.add(new Double (distanceY));
				}
				while (this.distance >= thisX-5 && this.distance <= thisX+5) {
					System.out.println("        "+ this.distance);
				}
			}
		}
	}

	/** @author: Kartik Misra
	 * 
	 * take data from arraylists to construct the coordinates of the blocks
	 * 
	 * @void
	 */
	public void constructCoordinates() {
		//		double result = distX.get(0);
		//		for (int i = 0; i<distX.size(); i++) {
		//			if (distX.get(i) < result) {
		//				result = distX.get(i);
		//			}
		//		}
		if (xPos.get(0) != null && yPos.get(0) != null) {
			block1X = xPos.get(0);
			block1Y = yPos.get(0);
			ProjectMain.block1X = block1X;
			ProjectMain.block1Y = block1Y;
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
