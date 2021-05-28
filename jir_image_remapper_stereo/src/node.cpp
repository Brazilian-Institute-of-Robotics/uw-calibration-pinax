/*
* Copyright (c) 2017 Jacobs University Robotics Group
* All rights reserved.
*
*
* Unless specified otherwise this code examples are released under 
* Creative Commons CC BY-NC-ND 4.0 license (free for non-commercial use). 
* Details may be found here: https://creativecommons.org/licenses/by-nc-nd/4.0/
*
*
* If you are interested in using this code commercially, 
* please contact us.
*
* THIS SOFTWARE IS PROVIDED BY Jacobs Robotics ``AS IS'' AND ANY
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL Jacobs Robotics BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Contact: robotics@jacobs-university.de
*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sstream>
#include <fstream>


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <jir_rectification_remap_lib/rectification_remapper.h>


using namespace std;
using namespace message_filters;


jir_rectification_remap_lib::RectificationRemapper remap_left;
jir_rectification_remap_lib::RectificationRemapper remap_right;

image_transport::CameraPublisher left_rect_pub;
sensor_msgs::CameraInfo left_info;

image_transport::CameraPublisher right_rect_pub;
sensor_msgs::CameraInfo right_info;


cv_bridge::CvImage left_rect;
sensor_msgs::Image left_rect_msg;

cv_bridge::CvImage right_rect;
sensor_msgs::Image right_rect_msg;


void process(const sensor_msgs::Image::ConstPtr& left, const sensor_msgs::Image::ConstPtr& right ) {
	cv_bridge::CvImageConstPtr cv_left;
	cv_bridge::CvImageConstPtr cv_right;
    try
    {
      cv_left = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::BGR8 );
	  cv_right = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::BGR8 );  
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ROS_INFO_STREAM("input image: " << cv_left->image.cols << "x" << cv_left->image.rows);

	//ROS_INFO_STREAM("converted");

	bool ok_left = remap_left (cv_left->image , left_rect.image );
	bool ok_right = remap_left (cv_right->image , right_rect.image );

	if(!ok_left || !ok_right) {
		ROS_ERROR_STREAM("error in remap, not right input image size?");
		return;
	}


	left_rect.header = cv_left->header;
	left_rect.encoding = sensor_msgs::image_encodings::BGR8;

	left_rect.toImageMsg(left_rect_msg);
	left_info.header = left_rect_msg.header;
	left_rect_pub.publish( left_rect_msg, left_info);

	right_rect.header = cv_right->header;
	right_rect.encoding = sensor_msgs::image_encodings::BGR8;

	right_rect.toImageMsg(right_rect_msg);
	right_info.header = right_rect_msg.header;
	right_rect_pub.publish( right_rect_msg, right_info);

}

int main(int argc, char *argv[])
{
	ROS_INFO_STREAM("node sarted");
	ros::init(argc, argv, "jir_image_remapper_stereo");

	ros::NodeHandle nh("~");

	std::string left_map, right_map;
	
	nh.getParam("left_map", left_map);
	nh.getParam("right_map", right_map);



	if(! remap_left.loadMap( left_map ) ) {
		ROS_ERROR_STREAM("error loading left map: " << left_map);
		return 1;
	}

	if(! remap_right.loadMap( right_map ) ) {
	ROS_ERROR_STREAM("error loading right map: " << right_map);
	return 1;
	}


	left_info =  remap_left.getCameraInfo();
	right_info =  remap_right.getCameraInfo();

	image_transport::ImageTransport it(nh);

	left_rect_pub = it.advertiseCamera("out_left_rect/image_raw", 1);
	right_rect_pub = it.advertiseCamera("out_right_rect/image_raw", 1);

	//image_transport::Subscriber sub = it.subscribe("in_left", "in_right",1, process);


	message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "in_left", 1);
	message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "in_right", 1);
	
	typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
	//TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_sub,right_sub , 1500);
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),left_sub,right_sub);
	sync.registerCallback(boost::bind(&process, _1, _2));


	ROS_INFO_STREAM("NODE READY");

	while(ros::ok()) {
		try {
			ros::spin();
		} catch ( boost::thread_interrupted& e ) {
			ROS_WARN_STREAM("caught thread_interrupted...");
		}
	}
	

	return 0;
}