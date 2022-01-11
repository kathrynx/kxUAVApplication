import java.util.*;
import java.lang.*;
import java.io.*;

public class Main {

	public static void main(String[] args) {
		while(true) {
			
			Scanner chooseAct = new Scanner(System.in);
			System.out.println("Select an option to test (1 or 2). \n 1: Drone Rotation. \n 2: Scan a Grid, \n 3: End test.");
			int option = chooseAct.nextInt();
			
			if(option == 3)
				break;
			
			double[] data = readFile();
			
			for(double a : data) {
				System.out.println(a);
			}
			
			if(option == 1) {
				createFile(droneRotation(data[0], data[1], data[2], data[3], data[4], data[5], data[6]));
				System.out.println();
				System.out.println();
				System.out.println("Test again.");
				
			} else if (option == 2) {
				Point[] ponts = new Point[(data.length/2)+1];
				int dex = 0;
				System.out.println("points:");
				for(int j = 0; j < data.length; j+=2) {
			    		ponts[dex] = new Point(data[j], data[j+1]);
			    		dex++;
			    		System.out.println("[" + data[j] + "," + data[j+1] + "]");
			    	}
				ponts[ponts.length-1] = new Point(data[0], data[1]);
				
				
				System.out.println();
				System.out.println();
				createFile(minArea(ponts));
				System.out.println();
				System.out.println();
				System.out.println("Test again.");
				
			} else if (option == 3) {
				break;
				
			} else {
				System.out.println("Invalid input.");
				System.out.println("Test again.");
			}
			
		}
		
	}

	public static double[] readFile() {
		try {
			Scanner sc = new Scanner(System.in);
			System.out.println("Input file name.");
			String strFile = sc.nextLine();
			File txtFile = new File(strFile);
			Scanner myReader = new Scanner(txtFile);
			
			String data = "";
			while(myReader.hasNextLine()) {
				String linData = myReader.nextLine();
				data += " " + linData;
			}
			myReader.close();
			
	        String[] arrOfStr = data.split(" ", 0);

	        
	        double[] arrDub = new double[arrOfStr.length-1];
	        for (int i = 1; i < arrOfStr.length; i++) {
	            Double aDub = Double.parseDouble(arrOfStr[i]);
	            arrDub[i-1] = aDub;
	        }
	        return arrDub;
	        
		} catch(FileNotFoundException e) {
			System.out.println("An error has occurred.");
			e.printStackTrace();
		}
		return null;
	}
	
	public static void createFile(double[][] ans) {
		try {
		      String strAns = "";
		      for(double[] ansRow : ans) {
		    	  for(double a : ansRow) {
		    		  strAns += a + " ";
		    	  }
		      }
		      
		      String fileName = "kxSubmission.txt";
		      FileWriter myWriter = new FileWriter(fileName);
		      myWriter.write(strAns);
		      myWriter.close();
		      System.out.println("Successfully wrote to the file.");
		    } catch (IOException e) {
		      System.out.println("An error occurred.");
		      e.printStackTrace();
		    }
	}
	
	/* 1. Drone Rotation
	The UAV needs to know which direction to orient itself to travel 
	to a certain point (because multicopters can only exert a linear 
	force up - i.e. thrust). Sometimes, we also want the opposite - 
	we want to predict where the drone will end up given a thrust and a rotation.
	Given a 3D starting point (x, y, z), the linear Euclidean distance upwards,
	and the rotation, in degrees roll, pitch and yaw, calculate the destination
	point (x1, y1, z1). Assume the drone starts facing north, with rotation (0, 0, 0).
	All coordinates are in ENU inertial frame. Positive rotations are counterclockwise
	around their respective axis. Note that a distance and no rotation (e.g. 1 0 0 0)
	will result in the drone moving upwards (e.g. 0 0 1).
	*/

	public static double[][] droneRotation (double x, double y, double z, double distance, double roll, double pitch, double yaw) {
		// starts at (x (east), y (north), z (up))
		// rotation: (roll, pitch, yaw)
		
		double position[][] = {{0, 0, distance}};
		
		double radRoll = Math.toRadians(roll);
		double radPitch = Math.toRadians(pitch);
		double radYaw = Math.toRadians(yaw);
		
		double[][] Rx = {
				{1, 0, 0},
				{0, Math.cos(radPitch), Math.sin(radPitch)},
				{0, -1*Math.sin(radPitch), Math.cos(radPitch)}
		};
		
		double[][] Ry = {
				{Math.cos(radRoll), 0, Math.sin(radRoll)},
				{0, 1, 0},
				{-1*Math.sin(radRoll), 0, Math.cos(radRoll)}
		};
		
		double[][] Rz = {
				{Math.cos(radYaw), Math.sin(radYaw), 0},
				{-1 * Math.sin(radYaw), Math.cos(radYaw), 0},
				{0, 0, 1}
		};
		
		position = multiplyMatrix(position, Rx);
		position = multiplyMatrix(position, Ry);
		position = multiplyMatrix(position, Rz);
		
		position[0][0] += x;
		position[0][1] += y;
		position[0][2] += z;
		
		System.out.println("final position: ");
		printMatrix(position);
		return position;

		
	}
	
	
	/*
	 * 2. Scan a Grid
	An important task of the drone is to scan a certain grid area for objects on the ground, 
	and it’s important to know how large this grid area would be. Given a list of x y coordinates, 
	find the rotation and the area of the smallest square (it may be rotated) containing all of 
	these coordinates. (Note that this is basically asking for a rotated bounding square).
	
	
	Input Format: A list of 2 integers (x, y), separated by newlines. 
	The rotated bounding square should contain all of these points in the minimum area possible.
	Output Format: A file with 2 integers - the area of the square, 
	and the minimum positive rotation of the square in degrees counterclockwise 
	(e.g. 45 instead of 135). These values should be rounded to the nearest whole number.
	Bounds: The square will have side lengths greater than 0. There will be at least 
	3 noncollinear points, but less than 10.
	 */
	
    public static double[][] minArea(Point[] points) {
    	
    	double area = 0;
    	double minArea = Integer.MAX_VALUE;
    	double angle = 0;
    	double minAngle = 0;
    	
    	
    	Point[] edges = convexHull(points, points.length);
    	System.out.println("edges length is: "+edges.length);
    	Point[] tempEdges = new Point[edges.length];
    	
    	
    	System.out.println("start\n");
    	for(int i = 0; i < edges.length; i++) {
    		if(edges[i]!=null) {
    			System.out.println(edges[i].x + " " + edges[i].y + "\n");
    		} else
    			System.out.println(i);
    	}
    	System.out.println("end\n");
    	
    	
    	for(int i = 0; i < edges.length-1; i++) {
	    		angle = Math.atan((edges[i+1].y - edges[i].y)/(edges[i+1].x - edges[i].x));
	    		for(int j = 0; j < edges.length; j++) {
	    			tempEdges[j] = rotatePoint(edges[j].x, edges[j].y, angle);
	    		}
	    		area = boundingSquare(tempEdges);
	    		if(area < minArea) {
	    			minArea = area;
	    			minAngle = angle;
	    		}
    		
    	}
    	
    	double degMinAng = Math.toDegrees(minAngle);
    	System.out.println("The minimized square has an area of: " + minArea);
    	System.out.println("The corresponding angle is: " + degMinAng);
    	double[][] ans = {{minArea, degMinAng}};
    	return ans;
    }
    
    
    
    
	
    public static double[][] multiplyMatrix(double[][] A, double[][] B) {
    	double[][] ans = new double[A.length][B[0].length];
    	double temp = 0;
    	
    	for(int i = 0; i < A.length; i++) {
    		for(int j = 0; j < B[0].length; j++) {
    			temp = 0;
    			for(int k = 0; k < B.length; k++) {
    				temp += A[i][k] * B[k][j];
    			}
    			ans[i][j] = temp;
    		}
    	}
    	
    	return ans;
    }
    
    public static Point rotatePoint(double x, double y, double angle) {
    	double[][] arrPont = {{x, y}};
    	double[][] rotMat = {
    			{Math.cos(angle), Math.sin(angle)},
    			{-1 * Math.sin(angle), Math.cos(angle)}
    	};
    	
    	arrPont = multiplyMatrix(arrPont, rotMat);
    	Point ans = new Point(arrPont[0][0], arrPont[0][1]);
    	
    	return ans;
    }

    public static void printMatrix(double[][] mat) {
    	for(int i = 0; i < mat.length; i++) {
    		for(int j = 0; j < mat[i].length; j++) {
    			System.out.print(mat[i][j] + ", ");
    		}
    		System.out.println();
    	}
    }
    
    public static int orientation(Point p1, Point p2,
            Point p3)
	{

		double val = (p2.y - p1.y) * (p3.x - p2.x) -
		(p2.x - p1.x) * (p3.y - p2.y);
		
		if (val == 0) return 0;  
		
		return (val > 0)? 1: 2;
	}



	public static Point[] convexHull(Point[] points, int n)
    {
		Point[] hullPoints = new Point[n];
        if (n < 3) return hullPoints;
      
        ArrayList<Point> hull = new ArrayList<Point>();
      
        int l = 0;
        for (int i = 1; i < n; i++) {

            if (points[i].x < points[l].x)
                l = i;
        }
      
        int p = l, q;
        do
        {
            hull.add(points[p]);
      
            q = (p + 1) % n;
             
            for (int i = 0; i < n; i++)
            {
               if (orientation(points[p], points[i], points[q])
                                                   == 2)
                   q = i;
            }
            p = q;
      
        } while (p != l); 
      
        Point[] newHullPoints = new Point[hull.size()];
        
        int i = 0;
        for (Point temp : hull) {
        	newHullPoints[i] = new Point(temp.x, temp.y);
        	i++;
            System.out.println("(" + temp.x + ", " + temp.y + ")");
        }
        
        
        return newHullPoints;
    }
    
    public static double boundingSquare(Point[] ponts) {
    	double xMax = Integer.MIN_VALUE;
    	double xMin = Integer.MAX_VALUE;
    	double yMax = Integer.MIN_VALUE;
    	double yMin = Integer.MAX_VALUE;
    	
    	for(Point pont : ponts) {
    		if(pont.x < xMin)
    			xMin = pont.x;
    		if(pont.x > xMax)
    			xMax = pont.x;
    		if(pont.y < yMin)
    			yMin = pont.y;
    		if(pont.y > yMax)
    			yMax = pont.y;
    	}
    	
    	if(xMax - xMin > yMax - yMin)
    		return (xMax - xMin) * (xMax - xMin);
    	else
    		return (yMax - yMin) * (yMax - yMin);
    }
       	
}
