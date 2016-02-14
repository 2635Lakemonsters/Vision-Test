package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDOutputDrive implements PIDOutput
{
	RobotDrive drive;
	
	public PIDOutputDrive(RobotDrive drive)
	{
		super();
		this.drive = drive;
	}

	@Override
	public void pidWrite(double output)
	{
		SmartDashboard.putNumber("Output to motors", output);
		drive.arcadeDrive(0, output);
	}

}
