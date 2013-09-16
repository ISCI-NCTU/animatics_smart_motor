package ca.etsmtl.capra;

import geometry_msgs.Twist;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import ca.etsmtl.capra.smartmotor.RobotDrive;

public class SmartMotor extends AbstractNodeMain
{
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////// Parameters //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String PARAM_NAME_N_MOTORS = "~n_motors";
	private static String PARAM_NAME_PORT_NAME = "~port";
	private static String PARAM_NAME_WATCHDOG_FREQ = "~watchdog_freq";
	
	private static String DEFAULT_PORT_NAME = "/dev/ttyUSB2002";
	private static int DEFAULT_N_MOTORS = 2;
	private static int DEFAULT_WATCHDOG_FREQ = 10;
	private int watchdogFreq = DEFAULT_WATCHDOG_FREQ;
	
	ParameterTree params;
	
    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Topics ///////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String TOPIC_NAME_ODOM = "~odom";
	private static String TOPIC_NAME_CMD_VEL = "~cmd_vel";
	
	Publisher<nav_msgs.Odometry> odomPublisher;
	Subscriber<geometry_msgs.Twist> cmdVelSubscriber;
	
	private ConnectedNode node;
	RobotDrive drive;
	
	float commandedLinearVelocity = 0;
	float commandedAngularVelocity = 0;
	boolean newVelocity = false;
	
	@Override
	public GraphName getDefaultNodeName()
	{
		return GraphName.of("capra_smartmotor");
	}
	
	private void initTopics()
	{
		odomPublisher = node.newPublisher(TOPIC_NAME_ODOM, nav_msgs.Odometry._TYPE);
		
		cmdVelSubscriber = node.newSubscriber(TOPIC_NAME_CMD_VEL, geometry_msgs.Twist._TYPE);
		cmdVelSubscriber.addMessageListener(new cmdVelListener());
	}
	
	private class cmdVelListener implements MessageListener<geometry_msgs.Twist>
	{
		@Override
		public void onNewMessage(Twist cmd_vel) 
		{
			commandedLinearVelocity = (float)cmd_vel.getLinear().getX();
			commandedAngularVelocity = (float)cmd_vel.getAngular().getZ();
			newVelocity = true;			
		}
	}
	
	private void initServices()
	{
		
	}
	
	private boolean connectToMotors()
	{	
		
		
		int nMotors = params.getInteger(PARAM_NAME_N_MOTORS, DEFAULT_N_MOTORS);
		String port = params.getString(PARAM_NAME_PORT_NAME, DEFAULT_PORT_NAME);
		
		drive = new RobotDrive(nMotors, port);
		if ( drive.openPort() )
		{
			drive.init();
			drive.setGlobalAccel(10);
			return true;			
		}
		
		return false;
	}
	

	@Override
	public void onStart(final ConnectedNode connectedNode)
	{
		node = connectedNode;
		params = node.getParameterTree();
		
		if ( connectToMotors() )
		{
			initTopics();
			initServices();
			
			// Watchdog loop
			watchdogFreq = params.getInteger(PARAM_NAME_WATCHDOG_FREQ, DEFAULT_WATCHDOG_FREQ);
			connectedNode.executeCancellableLoop(new CancellableLoop()
			{
				@Override
				protected void loop() throws InterruptedException
				{
					if ( newVelocity )
					{
						drive.setVelocity(commandedLinearVelocity, commandedAngularVelocity);
						newVelocity = false;
					}
					else
					{
						drive.setVelocity(0, 0);
					}
					
					Thread.sleep(1000 / watchdogFreq);					
				}
			});
		}
		else 
		{
			throw new RuntimeException("Error connecting to the motors");
		}
	}
}
