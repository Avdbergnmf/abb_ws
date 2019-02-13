// ROS Node for Geomagic Touch Haptic Device

#include <geomagic_touch_m/geomagic_touch_m.h>

//
// Geomagic Touch High-Level ROS Controller
//

void GeomagicTouch::init(GeomagicTouchState *state)
{
  // Init publishers
  geo_pos_pub = n.advertise<geometry_msgs::PoseStamped>("geo_pos_state_m", 10);
  // geo_vel_pub = n.advertise<geometry_msgs::TwistStamped>("geo_vel_state_m", 10);
	// geo_force_pub = n.advertise<geometry_msgs::WrenchStamped>("geo_force_state_m", 10);
  geo_button_pub = n.advertise<geomagic_touch_m::GeomagicButtonEvent>("geo_buttons_m", 10);
	// geo_haptic_state_pub = n.advertise<geomagic_touch_m::HapticState>("geo_haptic_state_m", 10);

  // Init subscribers
  // geo_effort_sub = n.subscribe("geo_control_effort_m", 10, &GeomagicTouch::effort_callback, this, ros::TransportHints().udp());
  // geo_enable_haptic_sub = n.subscribe("geo_enable_haptic_m", 10, &GeomagicTouch::enable_haptic, this, ros::TransportHints().udp());

	// Publish joint states for robot_state_publisher, and anyone else who wants them.
	joint_pub = n.advertise<sensor_msgs::JointState>("geo_joint_states", 1);

  // Init Geomagic state
  geo_state = state;
  geo_state->buttons[0] = 0;
	geo_state->buttons[1] = 0;
	geo_state->firstrun = 1;
	geo_state->feedback = 0;
	geo_state->buttons_prev[0] = 0;
	geo_state->buttons_prev[1] = 0;
	hduVector3Dd zeros(0, 0, 0);
	geo_state->velocity = zeros; 
	geo_state->pos_hist1 = zeros; 
	geo_state->pos_hist2 = zeros; 
	geo_state->lock = false;
	geo_state->geo_free = false;
	geo_state->lock_pos[0] = 0;
	geo_state->lock_pos[1] = M_PI/4;
	geo_state->lock_pos[2] = 0;

	buffer_size = 3;

	t = boost::circular_buffer<double>(buffer_size, 0.0);
	p_x = boost::circular_buffer<double>(buffer_size, 0.0);
	p_y = boost::circular_buffer<double>(buffer_size, 0.0);
	p_z = boost::circular_buffer<double>(buffer_size, 0.0);
	v_x = boost::circular_buffer<double>(buffer_size, 0.0);
	v_y = boost::circular_buffer<double>(buffer_size, 0.0);
	v_z = boost::circular_buffer<double>(buffer_size, 0.0);

  // Init Other variables
  en_h_count = 1;
}

void GeomagicTouch::enable_haptic(const std_msgs::Int32 en_h)
{
	// enable haptic feedback when incoming integer en_h=1,
  // else disable haptic feedback

  if(en_h.data == 1 && geo_state->feedback == 0)
	{
		geo_state->feedback = !(geo_state->feedback);
		//ROS_INFO("force feedback enabled");
	}
	else if (en_h.data == 0 && geo_state->feedback == 1)
	{
		geo_state->feedback = !(geo_state->feedback);
		//ROS_INFO("force feedback disabled");
	}
	else
	{

	}
}

void GeomagicTouch::effort_callback(const geometry_msgs::WrenchStamped control_effort)
{
	geo_state->effort[0] = control_effort.wrench.force.x;
	geo_state->effort[1] = control_effort.wrench.force.y;
	geo_state->effort[2] = control_effort.wrench.force.z;
}

void GeomagicTouch::publish_geo_state()
{
  // Publish current Geomagic Touch pose
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.stamp = ros::Time::now();
	pose_stamped.pose.position.x = geo_state->position[0]; //thetas[0]; // thetas to include indexing
	pose_stamped.pose.position.y = geo_state->position[1];
	pose_stamped.pose.position.z = geo_state->position[2];
	
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(geo_state->geo_mx[3][0], geo_state->geo_mx[3][1], geo_state->geo_mx[3][2]));
	transform.getBasis().setValue(geo_state->geo_mx[0][0], geo_state->geo_mx[1][0], geo_state->geo_mx[2][0],
								  							geo_state->geo_mx[0][1], geo_state->geo_mx[1][1], geo_state->geo_mx[2][1], 
								  							geo_state->geo_mx[0][2], geo_state->geo_mx[1][2], geo_state->geo_mx[2][2]);

	pose_stamped.pose.orientation.x = transform.getRotation()[0];
	pose_stamped.pose.orientation.y = transform.getRotation()[1];
	pose_stamped.pose.orientation.z = transform.getRotation()[2];
	pose_stamped.pose.orientation.w = transform.getRotation()[3];

	geo_pos_pub.publish(pose_stamped);

  // Publish current Geomagic Touch button states
  geomagic_touch_m::GeomagicButtonEvent button_event;
	button_event.grey_button = geo_state->buttons[0];
	button_event.white_button = geo_state->buttons[1];
	geo_state->buttons_prev[0] = geo_state->buttons[0];
	geo_state->buttons_prev[1] = geo_state->buttons[1];
	
  geo_button_pub.publish(button_event);


	// Joint state for rviz visualizer
	sensor_msgs::JointState joint_state;
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.name[0] = "waist";
	joint_state.position[0] = -geo_state->thetas[1];
	joint_state.name[1] = "shoulder";
	joint_state.position[1] = geo_state->thetas[2];
	joint_state.name[2] = "elbow";
	joint_state.position[2] = geo_state->thetas[3];
	joint_state.name[3] = "wrist1";
	joint_state.position[3] = -geo_state->thetas[4] + M_PI;
	joint_state.name[4] = "wrist2";
	joint_state.position[4] = -geo_state->thetas[5] - 3*M_PI/4;
	joint_state.name[5] = "wrist3";
	joint_state.position[5] = -geo_state->thetas[6] - M_PI;
	joint_pub.publish(joint_state);
}

//
// Communication with Geomagic Touch drivers
// Read joint- and Cartesian positions and apply forces or torques
//
HDCallbackCode HDCALLBACK geo_state_callback(void *pUserData) {

	// Find dt
	t.push_back(ros::Time::now().toSec());
	h = t.back() - t[buffer_size-2];
  if(h == 0)
  {
      h = 1/double(rate);
  }

	// gravity compensation terms in joint- and Cartesian space.
	hduVector3Dd gc;
	hduVector3Dd gcx;
	
	// start frame (loop frame at 1000Hz, forces/torques are
	// applied to the device at the end of each frame
	GeomagicTouchState *geo_state = static_cast<GeomagicTouchState *>(pUserData);
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
	  ROS_DEBUG("Updating calibration...");
	    hdUpdateCalibration(calibrationStyle);
	  }
	hdBeginFrame(hdGetCurrentDevice());
	
	// get gimbal angles, xyz positions of ee, joint positions and transform of ee (quaternions)
	hdGetDoublev(HD_CURRENT_POSITION, geo_state->position);
	p_x.push_back(geo_state->position[0]);
	p_y.push_back(geo_state->position[1]);
	p_z.push_back(geo_state->position[2]);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, geo_state->joints);
	hdGetDoublev(HD_CURRENT_TRANSFORM, geo_state->geo_mx);
	
	// convert read joint angles to DH-angles qm and write read xyz position in pm
	hduVector3Dd qm( -geo_state->joints[0], geo_state->joints[1], geo_state->joints[2]-geo_state->joints[1]-M_PI/2 );
	hduVector3Dd pm( geo_state->position[0], geo_state->position[1], geo_state->position[2] );

  // GRAVITY COMPENSATION
	
	// other parameters
	float Kg = 1.0;  // gain for Euler integration
	float l2 = 0.135; // length link 2
	float l3 = 0.135; // length link 3
		
  // gravity compensation joint space
  gc[0] = 0;
  gc[1] = 0.22331*(qm[1]-M_PI/2)-9.81*-0.038*cos(qm[1])-9.81*-0.0078*cos(qm[1]+qm[2]);
  gc[2] = -9.81*cos(qm[1]+qm[2])*-0.0078;
		
  // map gravity compensation in joint space to Cartesian space.
  // DH defined Jacobian is used, but base frame of the DH defined
  // robot is different than internal Omni base frame, so extra
  // mapping:
  //        x_DH = z_omni
  //        y_DH = x_omni
  //        z_DH = y_omni
  gcx[2] = -(cos(qm[0])*(l2*gc[2]*cos(qm[1]) - l3*gc[1]*cos(qm[1] + qm[2]) + l3*gc[2]*cos(qm[1] + qm[2])))/(l2*l3*sin(qm[2]));
  gcx[0] = -(sin(qm[0])*(l2*gc[2]*cos(qm[1]) - l3*gc[1]*cos(qm[1] + qm[2]) + l3*gc[2]*cos(qm[1] + qm[2])))/(l2*l3*sin(qm[2]));
  gcx[1] = (sin(qm[1] + qm[2])*(gc[1] - gc[2]))/(l2*sin(qm[2])) - (gc[2]*sin(qm[1]))/(l3*sin(qm[2]));
	
	effort[0] = geo_state->effort[0]; // klptx*geo_state->effort[0] + (1-klptz)*effort[0];
	effort[1] = geo_state->effort[1]; // klpty*geo_state->effort[1] + (1-klpty)*effort[1];
  effort[2] = geo_state->effort[2]; // klptz*geo_state->effort[2] + (1-klptz)*effort[2];
		
	// Force handler.
	if (geo_state->feedback == true) {
			geo_state->force = gcx+effort;
	}
	else {
		hduVector3Dd zeroforce(0, 0, 0);
			geo_state->force = gcx;//zeroforce;
	}

	// Apply control force to device
	hdSetDoublev(HD_CURRENT_FORCE, geo_state->force);
	
	// get button states
	int nButtons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	geo_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	geo_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	// end frame: after this the control force/torques are actually applied at the device
	hdEndFrame(hdGetCurrentDevice());

	// kill session if the device returns an error
	HDErrorInfo error;
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		hduPrintError(stderr, &error, "Error during main scheduler callback");
		if (hduIsSchedulerError(&error))
			return HD_CALLBACK_DONE;
	}

	// Joint states for the rviz visualization
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, geo_state->rot);
	
	float t[7] = { 0., geo_state->joints[0], geo_state->joints[1],
			geo_state->joints[2] - geo_state->joints[1], geo_state->rot[0],
			geo_state->rot[1], geo_state->rot[2] };
	for (int i = 0; i < 7; i++)
		geo_state->thetas[i] = t[i];

	return HD_CALLBACK_CONTINUE;
}

//
// Automatic Calibration of Geomagic Touch Device                               *
//
void HHD_Auto_Calibration() {
	int supportedCalibrationStyles;
	HDErrorInfo error;

	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
		calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
		ROS_INFO("HD_CALIBRATION_ENCODER_RESE..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
		calibrationStyle = HD_CALIBRATION_INKWELL;
		ROS_INFO("HD_CALIBRATION_INKWELL..");
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
		calibrationStyle = HD_CALIBRATION_AUTO;
		ROS_INFO("HD_CALIBRATION_AUTO..");
	}
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
	  do {
		hdUpdateCalibration(calibrationStyle);
		ROS_INFO("Calibrating.. (put stylus in well)");
		if (HD_DEVICE_ERROR(error = hdGetError())) {
			hduPrintError(stderr, &error, "Reset encoders reset failed.");
			break;
		}
	} while (hdCheckCalibration() != HD_CALIBRATION_OK);
	ROS_INFO("Calibration complete.");
	}
	if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT) {
	  ROS_INFO("Please place the device into the inkwell for calibration.");
	}
}

//
// Define publishing loop
//
void *ros_publish(void *ptr) {
	GeomagicTouch *geo = (GeomagicTouch *) ptr;
	ros::Rate loop_rate(rate);
	ros::AsyncSpinner spinner(2);
	spinner.start();

	while (ros::ok()) {
		geo->publish_geo_state();
		loop_rate.sleep();
	}
	return NULL;
}

//
// Main Function
//
int main(int argc, char** argv) {
	// Initialize Geomagic Touch device
	HDErrorInfo error;
	HHD hHD;
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		ROS_ERROR("Failed to initialize haptic device");
		return -1;
	}

	ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())) {
		ROS_ERROR("Failed to start the scheduler");
		return -1;
	}
	HHD_Auto_Calibration();

	// Initialize ROS
	ros::init(argc, argv, "Geomagic_Touch_control_node_m");
	GeomagicTouchState geo_state;
	GeomagicTouch geo;

	geo.init(&geo_state);
	hdScheduleAsynchronous(geo_state_callback, &geo_state, HD_MAX_SCHEDULER_PRIORITY);

	// Loop and publish
	pthread_t publish_thread;
	pthread_create(&publish_thread, NULL, ros_publish, (void*) &geo);
	pthread_join(publish_thread, NULL);

	ROS_INFO("Ending Session....");
	hdStopScheduler();
	hdDisableDevice(hHD);

	return 0;
}
