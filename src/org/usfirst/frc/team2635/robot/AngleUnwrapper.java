package org.usfirst.frc.team2635.robot;

import com.lakemonsters2635.sensor.interfaces.ISensor;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngleUnwrapper implements PIDSource
{
	Double accumulator;
	Double previousAngle;
	Double jumpTolerance;
	ISensor<Double> angleGetter;
	
	public AngleUnwrapper(Double jumpTolerance, ISensor<Double> angleGetter) {
		super();
		accumulator = 0.0;
		previousAngle = null;
		this.jumpTolerance = jumpTolerance;
		this.angleGetter = angleGetter;
	}

	@Override
	public double pidGet() {
		if(previousAngle == null)
		{
			//Initialization
			previousAngle = angleGetter.sense();
			return accumulator;
		}
		Double currentAngle = angleGetter.sense();
		Double delta = previousAngle - currentAngle;
		SmartDashboard.putNumber("Delta", delta);
		if(delta > jumpTolerance)
		{
			accumulator -= delta - jumpTolerance*2;
		}
		else if (delta < -jumpTolerance)
		{
			accumulator -= delta + jumpTolerance*2;
		}
		else
		{
			accumulator -= delta;
		}
		previousAngle = currentAngle;

		return accumulator;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType()
	{
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}


	
}
