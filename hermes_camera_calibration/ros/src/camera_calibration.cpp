#include <hermes_camera_calibration/camera_calibration.h>

CameraCalibration::CameraCalibration(ros::NodeHandle nh)
: node_handle_(nh)
{
	// subscribers
	input_marker_detection_sub_ = node_handle_.subscribe("input_marker_detections", 1, &CameraCalibration::inputCallback, this);

	// dynamic reconfigure
	dynamic_reconfigure_server_.setCallback(boost::bind(&CameraCalibration::dynamicReconfigureCallback, this, _1, _2));

	base_translation_.setZero();
}

CameraCalibration::~CameraCalibration()
{
}

/// callback for the incoming pointcloud data stream
void CameraCalibration::inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg)
{
	//std::cout << "Receiving detection of coordinate system..." << std::endl;

	// search for markers
	int marker_count = 0;
	tf::Point avg_translation;
	tf::Quaternion avg_orientation;
	for (unsigned int i=0; i<input_marker_detections_msg->detections.size(); i++)
	{
		if (input_marker_detections_msg->detections[i].label.compare("tag_1") == 0 || input_marker_detections_msg->detections[i].label.compare("tag_3") == 0)
		{
			// update average coordinate system of marker
			tf::Point new_translation;
			tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, new_translation);
			tf::Quaternion new_orientation;
			tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, new_orientation);
			if (marker_count==0)
			{
				avg_translation = new_translation;
				avg_orientation = new_orientation;
			}
			else
			{
				avg_translation += new_translation;
				avg_orientation += new_orientation;
			}
			++marker_count;
		}
	}

	// update averaged coordinate system
	if (marker_count > 0)
	{
		avg_translation /= (double)marker_count;
		avg_orientation /= (double)marker_count;
		avg_orientation.normalize();
		if (base_translation_.isZero())
		{
			// use value directly on first message
			base_translation_ = avg_translation;
			base_orientation_ = avg_orientation;
		}
		else
		{
			// update value
			base_translation_ = (1.0-update_rate_) * base_translation_ + update_rate_ * avg_translation;
			base_orientation_.setW((1.0-update_rate_) * base_orientation_.getW() + update_rate_ * avg_orientation.getW());
			base_orientation_.setX((1.0-update_rate_) * base_orientation_.getX() + update_rate_ * avg_orientation.getX());
			base_orientation_.setY((1.0-update_rate_) * base_orientation_.getY() + update_rate_ * avg_orientation.getY());
			base_orientation_.setZ((1.0-update_rate_) * base_orientation_.getZ() + update_rate_ * avg_orientation.getZ());
		}

		// publish new coordinate transformation
		tf::StampedTransform transform_camera_optframe_to_link;
		try
		{
			transform_listener_.lookupTransform(input_marker_detections_msg->header.frame_id, "/cam3d_link", input_marker_detections_msg->header.stamp, transform_camera_optframe_to_link);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}
		tf::Transform marker_transform(base_orientation_, base_translation_);
		transform_broadcaster_.sendTransform(tf::StampedTransform(marker_transform.inverse()*transform_camera_optframe_to_link, input_marker_detections_msg->header.stamp, parent_frame_for_camera_, "/cam3d_link"));
	}
}

void CameraCalibration::dynamicReconfigureCallback(hermes_camera_calibration::CameraCalibrationConfig &config, uint32_t level)
{
	update_rate_ = config.update_rate;
	parent_frame_for_camera_ = config.parent_frame_for_camera;

	std::cout << "Reconfigure request with\n  update_rate=" << update_rate_ << "\n  parent_frame_for_camera=" << parent_frame_for_camera_ << "\n";
}

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "camera_calibration");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraCalibration
	CameraCalibration objectRecording(nh);

	ros::spin();

	return (0);
}
