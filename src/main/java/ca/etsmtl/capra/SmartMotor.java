package ca.etsmtl.capra;

import java.util.Observable;
import java.util.Observer;

import geometry_msgs.Twist;
import nav_msgs.Odometry;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import ca.etsmtl.capra.smartmotor.RobotDrive;
import ca.etsmtl.capra.smartmotor.io.MotorController;

public class SmartMotor extends AbstractNodeMain
{
	////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////// Parameters //////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String PARAM_NAME_N_MOTORS = "~n_motors";
	private static String PARAM_NAME_PORT_NAME = "~port";
	private static String PARAM_NAME_WATCHDOG_FREQ = "~watchdog_rate";
	private static String PARAM_NAME_COVARIANCE = "~covariance";
	
	private static String DEFAULT_PORT_NAME = "/dev/ttyUSB2002";
	private static int DEFAULT_N_MOTORS = 2;
	private static int DEFAULT_WATCHDOG_RATE = 5;
	private static double DEFAULT_COVARIANCE = 100.0;
	
	private int nMotors;
	private String portName;
	private int watchdogFreq = DEFAULT_WATCHDOG_RATE;	
	private double covariance = 100.0;
	
	ParameterTree params;
	
    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Topics ///////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private static String TOPIC_NAME_ODOM = "~odom";
	private static String TOPIC_NAME_CMD_VEL = "~cmd_vel";
	
	private Publisher<nav_msgs.Odometry> odomPublisher;
	private Subscriber<geometry_msgs.Twist> cmdVelSubscriber;
	
    ////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// Other ////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
	private ConnectedNode node;
	private RobotDrive drive;
	
	private CommandType commandType = CommandType.CMD_VEL;
	
	private float commandedLinearVelocity = 0;
	private float commandedAngularVelocity = 0;
	private long lastVelocityUpdate = 0;
	
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
	
	private void loadParams ( )
	{
		params = node.getParameterTree();
		
		nMotors = params.getInteger(PARAM_NAME_N_MOTORS, DEFAULT_N_MOTORS);
		portName = params.getString(PARAM_NAME_PORT_NAME, DEFAULT_PORT_NAME);
		watchdogFreq = params.getInteger(PARAM_NAME_WATCHDOG_FREQ, DEFAULT_WATCHDOG_RATE);
		covariance = params.getDouble(PARAM_NAME_COVARIANCE, DEFAULT_COVARIANCE);
	}
	
	private class cmdVelListener implements MessageListener<geometry_msgs.Twist>
	{
		@Override
		public void onNewMessage(Twist cmd_vel) 
		{
			float newLinear = (float)cmd_vel.getLinear().getX();
			float newAngular = (float)cmd_vel.getAngular().getZ();
			
			lastVelocityUpdate = System.currentTimeMillis();
			commandType = CommandType.CMD_VEL;
			
			if ( commandedLinearVelocity != newLinear || commandedAngularVelocity != newAngular )
			{
				commandedLinearVelocity = newLinear;
				commandedAngularVelocity = newAngular;
				drive.setVelocity(commandedLinearVelocity, commandedAngularVelocity);
			}
		}
	}
	
	private void initServices()
	{
		
	}
	
	private boolean connectToMotors()
	{		
		drive = new RobotDrive(nMotors, portName);
		if ( drive.openPort() )
		{
			drive.init();
			drive.setGlobalAccel(10);
			return true;			
		}
		
		return false;
	}
	
	private void initWatchdog ( )
	{
		node.executeCancellableLoop(new CancellableLoop()
		{
			@Override
			protected void loop() throws InterruptedException
			{
				if ( drive.isMoving() && System.currentTimeMillis() - lastVelocityUpdate > 2 * (1000 / watchdogFreq)
					 && commandType == CommandType.CMD_VEL )
				{
					commandedLinearVelocity = 0;
					commandedAngularVelocity = 0;
					drive.setVelocity(commandedLinearVelocity, commandedAngularVelocity);
				}
				
				Thread.sleep(1000 / watchdogFreq);		
			}
		});
	}
	
	private void initTelemetryPublisher()
	{
		MotorController.getInstance().getInputControl().addObserver(new Observer() 
		{			
			@Override
			public void update(Observable arg0, Object arg1)
			{
				/*
				double[] pos = drive.getPosition();
				
				Odometry odom = node.getServiceRequestMessageFactory().newFromType(nav_msgs.Odometry._TYPE);
				
				// Position
				geometry_msgs.Point position = odom.getPose().getPose().getPosition();
				position.setX(pos[0]);
				position.setY(pos[1]);
				position.setZ(0);
				
				// Covariance de la position
				//Todo: Trouver la vraie covariance un jour
				odom.getPose().setCovariance(new double[]{ covariance, 0, 0, 0, 0, 0,
						                                   0, covariance, 0, 0, 0, 0,
						                                   0, 0, covariance, 0, 0, 0,
						                                   0, 0, 0, covariance, 0, 0,
						                                   0, 0, 0, 0, covariance, 0,
						                                   0, 0, 0, 0, 0, covariance});
				
				// Velocity
				//Todo: Modifier pour avoir la vraie valeur
				odom.getTwist().getTwist().getLinear().setX(commandedLinearVelocity);
				odom.getTwist().getTwist().getAngular().setZ(commandedAngularVelocity);
				
				// Covariance de la velocité
				//Todo Trouver la vraie covariance
				odom.getTwist().setCovariance(new double[]{ covariance, 0, 0, 0, 0, 0,
									                        0, covariance, 0, 0, 0, 0,
									                        0, 0, covariance, 0, 0, 0,
									                        0, 0, 0, covariance, 0, 0,
									                        0, 0, 0, 0, covariance, 0,
									                        0, 0, 0, 0, 0, covariance});*/
			}
		});
	}

	private void init ( )
	{
		initTopics();
		initServices();
		initWatchdog();
		initTelemetryPublisher();
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode)
	{
		node = connectedNode;
		loadParams();
		
		if ( connectToMotors() )
			init();
		else 
			throw new RuntimeException("Error connecting to the motors");
	}
}
