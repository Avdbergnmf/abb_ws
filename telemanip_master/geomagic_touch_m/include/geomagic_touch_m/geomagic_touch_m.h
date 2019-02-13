#ifndef GEOMAGIC_TOUCH_M_H
#define GEOMAGIC_TOUCH_M_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

#include <tf/transform_listener.h>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>

#include <pthread.h>
#include <math.h>
#include <geomagic_touch_m/GeomagicButtonEvent.h>
#include <geomagic_touch_m/HapticState.h>
#include <boost/circular_buffer.hpp>


// General declarations

hduVector3Dd effort;
double ti_v[2] = {0.0, 0.0};
double vel_prev[3] = {0.0, 0.0, 0.0};
int calibrationStyle;
boost::circular_buffer<double> t, p_x, p_y, p_z, v_x, v_y, v_z;
double h, vf_x, vf_y, vf_z;
std::vector<double> K;
int buffer_size;

// General settings

int rate = 1000;
int select_joint = 0; // 0 for cartesian space, 1 for joint space


struct GeomagicTouchState
{
  hduVector3Dd position;
	hduMatrix geo_mx;
	hduVector3Dd velocity; 
	hduVector3Dd pos_hist1; 
	hduVector3Dd pos_hist2;
	hduVector3Dd rot;
	hduVector3Dd joints;
	hduVector3Dd force; 
  hduVector3Dd effort;
	float thetas[6];
	// float pose_history[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	// float pose_reset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	float pub_effort[3];
	float joint_vel[3];
	int buttons[2];
	int firstrun;
	int feedback;
	int buttons_prev[2];
	bool lock;
	bool geo_free;
	hduVector3Dd lock_pos;
};

class GeomagicTouch
{
    public:
        ros::NodeHandle n;
        ros::Publisher geo_pos_pub, geo_vel_pub, geo_force_pub, geo_button_pub, geo_haptic_state_pub;
        ros::Subscriber geo_effort_sub, geo_enable_haptic_sub;

				ros::Publisher joint_pub;

        GeomagicTouchState *geo_state;

        int en_h_count;
        geomagic_touch_m::HapticState haptic_state_m;

        void init(GeomagicTouchState *state);
        void enable_haptic(const std_msgs::Int32 en_h);
        void effort_callback(const geometry_msgs::WrenchStamped control_efort);
        void publish_geo_state();
};



#endif //GEOMAGIC_TOUCH_M_H
