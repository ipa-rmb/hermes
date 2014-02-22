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

	// search for marker No. 1
	for (unsigned int i=0; i<input_marker_detections_msg->detections.size(); i++)
	{
		if (input_marker_detections_msg->detections[i].label.compare("tag_3") == 0)
		{
			// update average coordinate system of marker
			tf::Point new_translation;
			tf::pointMsgToTF(input_marker_detections_msg->detections[i].pose.pose.position, new_translation);
			tf::Quaternion new_orientation;
			tf::quaternionMsgToTF(input_marker_detections_msg->detections[i].pose.pose.orientation, new_orientation);
			if (base_translation_.isZero())
			{
				// use value directly on first message
				base_translation_ = new_translation;
				base_orientation_ = new_orientation;
			}
			else
			{
				// update value
				base_translation_ = (1.0-update_rate_) * base_translation_ + update_rate_ * new_translation;
				base_orientation_.setW((1.0-update_rate_) * base_orientation_.getW() + update_rate_ * new_orientation.getW());
				base_orientation_.setX((1.0-update_rate_) * base_orientation_.getX() + update_rate_ * new_orientation.getX());
				base_orientation_.setY((1.0-update_rate_) * base_orientation_.getY() + update_rate_ * new_orientation.getY());
				base_orientation_.setZ((1.0-update_rate_) * base_orientation_.getZ() + update_rate_ * new_orientation.getZ());
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
