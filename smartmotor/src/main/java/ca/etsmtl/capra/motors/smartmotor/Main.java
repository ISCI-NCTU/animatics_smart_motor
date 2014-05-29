package ca.etsmtl.capra.motors.smartmotor;

import org.ros.RosRun;

public class Main
{
	// Pour partir le noeud en debug dans Eclipse
	public static void main(String[] args) throws Exception
	{
		RosRun.main(new String[]{"ca.etsmtl.capra.motors.smartmotor.SmartMotor"});
	}

}
