package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;
import com.lakemonsters2635.sensor.interfaces.ISensor;

public class SensorNavxAngle implements ISensor<Double>
{
	AHRS navx;
	public SensorNavxAngle(AHRS navx) 
	{
		super();
		this.navx = navx;
	}
	@Override
	public Double sense() 
	{
		return navx.getAngle();
	}

}
