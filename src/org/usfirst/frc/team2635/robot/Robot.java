
package org.usfirst.frc.team2635.robot;


import java.util.Comparator;
import java.util.Vector;

import org.usfirst.frc.team2635.robot.Robot.ParticleReport;

import com.kauailabs.navx.frc.AHRS;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.ColorMode;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.PointFloat;
import com.ni.vision.NIVision.ROI;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
   
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
		double Area;
		double AreaByImageArea;
		double BoundingRectLeft;
		double BoundingRectTop;
		double BoundingRectWidth;
		double BoundingRectHeight;
		
		public int compareTo(ParticleReport r)
		{
			return (int)(r.AreaByImageArea - this.AreaByImageArea);
		}
		
		public int compare(ParticleReport r1, ParticleReport r2)
		{
			return (int)(r1.AreaByImageArea - r2.AreaByImageArea);
		}
	};
	final int REAR_RIGHT_CHANNEL = 3;
	final int FRONT_RIGHT_CHANNEL = 4;
	final int REAR_LEFT_CHANNEL = 1;
	final int FRONT_LEFT_CHANNEL = 2;
	SendableChooser cameraView;
	String BINARY = "binary";
	String COLOR = "color";
    int session;
    Image colorFrame;
    Image binaryFrame;
    Image overlayFrame;
	NIVision.Range RETRO_HUE_RANGE = new NIVision.Range(0, 5);	//Default hue range for yellow tote
	NIVision.Range RETRO_SAT_RANGE = new NIVision.Range(0, 10);	//Default saturation range for yellow tote
	NIVision.Range RETRO_VAL_RANGE = new NIVision.Range(250, 255);	//Default value range for yellow tote
	double MIN_RECT_WIDTH = 0.0;
	double MAX_RECT_WIDTH = 200.0;
	double MIN_RECT_HEIGHT = 0.0;
	double MAX_RECT_HEIGHT = 200.0;
	int CAMERA_RESOLUTION_X = 640;
	//I have no clue what a scale range is. Calibrate it through dashboard.
	NIVision.RangeFloat SCALE_RANGE = new NIVision.RangeFloat(0.0, 200.0);
	int MIN_MATCH_SCORE = 0;
	
	double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
	double LONG_RATIO = 20.0/12.0; //Retro long side = 20 / Retro height = 12 
	double SCORE_MIN = 75.0;  //Minimum score to be considered a match
	double VIEW_ANGLE = 64.0; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
	NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
	NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
	NIVision.RangeFloat rectAngleRanges[] = {new NIVision.RangeFloat(60.0, 110.0)};
	CANTalon rearRight;
	CANTalon frontRight;
	CANTalon rearLeft;
	CANTalon frontLeft;
	RobotDrive drive;
    Joystick joystick;
    PIDController pid;
    AHRS navx;
    AngleUnwrapper unwrapper;
    double angleToTarget = 0.0;
    public void robotInit() {
    	SmartDashboard.putNumber("P", 0);
    	SmartDashboard.putNumber("I", 0);
    	SmartDashboard.putNumber("D", 0);
		rearRight = new CANTalon(REAR_RIGHT_CHANNEL);
    	frontRight = new CANTalon(FRONT_RIGHT_CHANNEL);
    	rearLeft = new CANTalon(REAR_LEFT_CHANNEL);
    	frontLeft = new CANTalon(FRONT_LEFT_CHANNEL);
		drive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	joystick = new Joystick(0);
    	navx = new AHRS(SerialPort.Port.kMXP);
    	unwrapper = new AngleUnwrapper(180.0, new SensorNavxAngle(navx));
    	pid = new PIDController(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"),unwrapper, new PIDOutputDrive(drive));
	    overlayFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
	    
        colorFrame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
        binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
        NIVision.IMAQdxStartAcquisition(session);
		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);

        
		SmartDashboard.putNumber("Retro hue min", RETRO_HUE_RANGE.minValue);
		SmartDashboard.putNumber("Retro hue max", RETRO_HUE_RANGE.maxValue);
		SmartDashboard.putNumber("Retro sat min", RETRO_SAT_RANGE.minValue);
		SmartDashboard.putNumber("Retro sat max", RETRO_SAT_RANGE.maxValue);
		SmartDashboard.putNumber("Retro val min", RETRO_VAL_RANGE.minValue);
		SmartDashboard.putNumber("Retro val max", RETRO_VAL_RANGE.maxValue);
		cameraView = new SendableChooser();
		cameraView.addDefault("Color", COLOR);
		cameraView.addObject("Binary", BINARY);
		SmartDashboard.putData("Camera View",cameraView);
//		SmartDashboard.putNumber("Min Rect Width", MIN_RECT_WIDTH);
//		SmartDashboard.putNumber("Max Rect Width", MAX_RECT_WIDTH);
//		SmartDashboard.putNumber("Min Rect Height", MIN_RECT_HEIGHT);
//		SmartDashboard.putNumber("Max Rect Height", MAX_RECT_HEIGHT);
//		
//		SmartDashboard.putNumber("Scale Range Min", 0);
//		SmartDashboard.putNumber("Scale Range Max", 200);
//		SmartDashboard.putNumber("Min Match Score", 0);
//		
		SmartDashboard.putNumber("Area min %", AREA_MINIMUM);


    }
    

    public void autonomousInit() {
 
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    
    }
    public double pixelCoordnateToAimingCoordnate(double pixelCoordnate, double resolution)
    {
    	//Shift pixelCoordnate to the right by half the resolution, effectively transforming the plane of the coordnate from 0 on the lefthand side to -resolution/2 on the lefthand side
    	//Divide by half the resolution to get the ratio between the coordnate position and its distance to 0.
    	return (pixelCoordnate - resolution / 2.0) / (resolution / 2.0);
    }
    public double aimingCoordnateToAngle(double aimingCoordnate, double viewingAngle)
    {
    	return aimingCoordnate * viewingAngle/2.0;
    }
    public void teleopPeriodic() {
    	//Grab a frame from the camera
        NIVision.IMAQdxGrab(session, colorFrame, 1);				
        RETRO_HUE_RANGE.minValue = (int)SmartDashboard.getNumber("Retro hue min", RETRO_HUE_RANGE.minValue);
		RETRO_HUE_RANGE.maxValue = (int)SmartDashboard.getNumber("Retro hue max", RETRO_HUE_RANGE.maxValue);
		RETRO_SAT_RANGE.minValue = (int)SmartDashboard.getNumber("Retro sat min", RETRO_SAT_RANGE.minValue);
		RETRO_SAT_RANGE.maxValue = (int)SmartDashboard.getNumber("Retro sat max", RETRO_SAT_RANGE.maxValue);
		RETRO_VAL_RANGE.minValue = (int)SmartDashboard.getNumber("Retro val min", RETRO_VAL_RANGE.minValue);
		RETRO_VAL_RANGE.maxValue = (int)SmartDashboard.getNumber("Retro val max", RETRO_VAL_RANGE.maxValue);
		AREA_MINIMUM = SmartDashboard.getNumber("Area min %");
		//MIN_RECT_WIDTH = SmartDashboard.getNumber("Min Rect Width", MIN_RECT_WIDTH);
		//MAX_RECT_WIDTH = SmartDashboard.getNumber("Max Rect Width", MAX_RECT_WIDTH);
		//MIN_RECT_HEIGHT = SmartDashboard.getNumber("Min Rect Height", MIN_RECT_HEIGHT);
		//MAX_RECT_HEIGHT= SmartDashboard.getNumber("Max Rect Height", MAX_RECT_HEIGHT);
		
		//SCALE_RANGE.minValue = (int)SmartDashboard.getNumber("Scale Range Min");
		//SCALE_RANGE.maxValue = (int)SmartDashboard.getNumber("Scale Range Max");
		//MIN_MATCH_SCORE = (int)SmartDashboard.getNumber("Min Match Score");
        //Look at the color frame for colors that fit the range. Colors that fit the range will be transposed as a 1 to the binary frame.
		NIVision.imaqColorThreshold(binaryFrame, colorFrame, 255, ColorMode.HSV, RETRO_HUE_RANGE, RETRO_SAT_RANGE, RETRO_VAL_RANGE);
		//Send the binary image to the cameraserver
		if(cameraView.getSelected() == BINARY)
		{
			CameraServer.getInstance().setImage(binaryFrame);
		}

		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);		
		
        NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

        //        NIVision.RectangleDescriptor rectangleDescriptor = new NIVision.RectangleDescriptor(MIN_RECT_WIDTH, MAX_RECT_WIDTH, MIN_RECT_HEIGHT, MAX_RECT_HEIGHT);
//        
//        //I don't know
//        NIVision.CurveOptions curveOptions = new NIVision.CurveOptions(NIVision.ExtractionMode.NORMAL_IMAGE, 0, NIVision.EdgeFilterSize.NORMAL, 0, 1, 1, 100, 1,1);
//        NIVision.ShapeDetectionOptions shapeDetectionOptions = new NIVision.ShapeDetectionOptions(1, rectAngleRanges, SCALE_RANGE, MIN_MATCH_SCORE);
//        NIVision.ROI roi = NIVision.imaqCreateROI();
//
//        NIVision.DetectRectanglesResult result = NIVision.imaqDetectRectangles(binaryFrame, rectangleDescriptor, curveOptions, shapeDetectionOptions, roi);
//        //Dummy rectangle to start
//        
//        NIVision.RectangleMatch bestMatch = new NIVision.RectangleMatch(new PointFloat[]{new PointFloat(0.0, 0.0)}, 0, 0, 0, 0);
//        
//        //Find the best matching rectangle
//        for(NIVision.RectangleMatch match : result.array)
//        {
//        	if(match.score > bestMatch.score)
//        	{
//        		bestMatch = match;
//        	}
//        }
//        SmartDashboard.putNumber("Rect height", bestMatch.height);
//        SmartDashboard.putNumber("Rect Width", bestMatch.width);
//        SmartDashboard.putNumber("Rect rotation", bestMatch.rotation);
//        SmartDashboard.putNumber("Rect Score", bestMatch.score);
        
        //Report how many particles there are
		int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
		SmartDashboard.putNumber("Masked particles", numParticles);
		
        
		if(numParticles > 0)
		{
			//Measure particles and sort by particle size
			Vector<ParticleReport> particles = new Vector<ParticleReport>();
			for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
			{
				ParticleReport par = new ParticleReport();
				par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
				par.AreaByImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
				par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
				par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
				par.BoundingRectHeight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_HEIGHT);
				par.BoundingRectWidth = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_WIDTH);
				particles.add(par);

			}
			particles.sort(null);

			//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
			//for the reader. Note that this scores and reports information about a single particle (single L shaped target). To get accurate information 
			//about the location of the tote (not just the distance) you will need to correlate two adjacent targets in order to find the true center of the tote.
//			scores.Aspect = AspectScore(particles.elementAt(0));
//			SmartDashboard.putNumber("Aspect", scores.Aspect);
//			scores.Area = AreaScore(particles.elementAt(0));
//			SmartDashboard.putNumber("Area", scores.Area);
//			boolean isTote = scores.Aspect > SCORE_MIN && scores.Area > SCORE_MIN;
//
			ParticleReport bestParticle = particles.elementAt(0);
		    NIVision.Rect particleRect = new NIVision.Rect((int)bestParticle.BoundingRectTop, (int)bestParticle.BoundingRectLeft, (int)bestParticle.BoundingRectHeight, (int)bestParticle.BoundingRectWidth);
		    
		    //NIVision.imaqOverlayRect(colorFrame, particleRect, new NIVision.RGBValue(0, 0, 0, 255), DrawMode.PAINT_VALUE, "a");//;(colorFrame, colorFrame, particleRect, DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 20);
		    NIVision.imaqDrawShapeOnImage(colorFrame, colorFrame, particleRect, NIVision.DrawMode.DRAW_VALUE, ShapeMode.SHAPE_RECT, 0.0f);
		    SmartDashboard.putNumber("Rect Top", bestParticle.BoundingRectTop);
		    SmartDashboard.putNumber("Rect Left", bestParticle.BoundingRectLeft);
		    SmartDashboard.putNumber("Rect Width", bestParticle.BoundingRectWidth);
		    SmartDashboard.putNumber("Area by image area", bestParticle.AreaByImageArea);
		    SmartDashboard.putNumber("Area", bestParticle.Area);
		    double bestParticleMidpoint = bestParticle.BoundingRectLeft + bestParticle.BoundingRectWidth/2.0;
		    double bestParticleMidpointAimingCoordnates = pixelCoordnateToAimingCoordnate(bestParticleMidpoint, CAMERA_RESOLUTION_X);
		    angleToTarget = aimingCoordnateToAngle(bestParticleMidpointAimingCoordnates, VIEW_ANGLE);
		    
		}
		else
		{
			angleToTarget = 0.0;
		}

        if(cameraView.getSelected() == COLOR)
        {
        	CameraServer.getInstance().setImage(colorFrame);
        }
	    SmartDashboard.putNumber("Angle to target", angleToTarget);

        if(joystick.getRawButton(1))
        {
        	SmartDashboard.putBoolean("PIDDrive", true);
        	pid.setPID(SmartDashboard.getNumber("P"), SmartDashboard.getNumber("I"), SmartDashboard.getNumber("D"));
        	double setPoint = unwrapper.pidGet() - angleToTarget;
        	SmartDashboard.putNumber("Set point", setPoint);
        	pid.setSetpoint(setPoint);
        	pid.enable();
  
        }
        else
        {
        	SmartDashboard.putBoolean("PIDDrive", true);
        	pid.disable();
        	drive.arcadeDrive(-joystick.getRawAxis(1), -joystick.getRawAxis(0));
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
