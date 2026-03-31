#include "realsense_gazebo_plugin/gazebo_ros_realsense.h"
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace {
std::string extractCameraName(const std::string &name);
sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image &image,
                                   float horizontal_fov);
}

namespace gazebo {
// Register the plugin
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRealsense)

GazeboRosRealsense::GazeboRosRealsense() {}

GazeboRosRealsense::~GazeboRosRealsense() {
  ROS_DEBUG_STREAM_NAMED("realsense_camera", "Unloaded");
}

void GazeboRosRealsense::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable "
                     "to load plugin. "
                     << "Load the Gazebo system plugin "
                        "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                        "package)");
    return;
  }
  ROS_INFO("Realsense Gazebo ROS plugin loading.");

  RealSensePlugin::Load(_model, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->GetHandle());

  // initialize camera_info_manager
  this->camera_info_manager_.reset(
      new camera_info_manager::CameraInfoManager(*this->rosnode_, this->GetHandle()));

  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  // set 'png' compression format for depth images
  // default functional parameters for compressed_image_transport to have lossless png compression
  rosnode_->setParam(rosnode_->resolveName(cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name) + "/compressed/format", "png");
  rosnode_->setParam(rosnode_->resolveName(cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name) + "/compressed/png_level", 1);

  this->color_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[COLOR_CAMERA_NAME].topic_name, 2);
  if (this->ired1Cam) {
    this->ir1_pub_ = this->itnode_->advertiseCamera(
        cameraParamsMap_[IRED1_CAMERA_NAME].topic_name, 2);
  }
  if (this->ired2Cam) {
    this->ir2_pub_ = this->itnode_->advertiseCamera(
        cameraParamsMap_[IRED2_CAMERA_NAME].topic_name, 2);
  }
  this->depth_pub_ = this->itnode_->advertiseCamera(
      cameraParamsMap_[DEPTH_CAMERA_NAME].topic_name, 2);
  if (pointCloud_)
  {
    this->pointcloud_pub_ =
        this->rosnode_->advertise<sensor_msgs::PointCloud2>(pointCloudTopic_, 2, false);
  }
}

void GazeboRosRealsense::OnNewFrame(const rendering::CameraPtr cam,
                                    const transport::PublisherPtr pub) {
  common::Time current_time = this->world->SimTime();

  // identify camera
  std::string camera_id = extractCameraName(cam->Name());
  std::map<std::string, image_transport::CameraPublisher *>
      camera_publishers = {
          {COLOR_CAMERA_NAME, &(this->color_pub_)},
      };
  if (this->ired1Cam) {
    camera_publishers[IRED1_CAMERA_NAME] = &(this->ir1_pub_);
  }
  if (this->ired2Cam) {
    camera_publishers[IRED2_CAMERA_NAME] = &(this->ir2_pub_);
  }
  const auto it = camera_publishers.find(camera_id);
  if (it == camera_publishers.end()) return;
  const auto image_pub = it->second;

  // copy data into image
  this->image_msg_.header.frame_id =
      this->cameraParamsMap_[camera_id].optical_frame;
  this->image_msg_.header.stamp.sec = current_time.sec;
  this->image_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  const std::map<std::string, std::string> supported_image_encodings = {
      {"RGB_INT8", sensor_msgs::image_encodings::RGB8},
      {"L_INT8", sensor_msgs::image_encodings::TYPE_8UC1}};
  const auto pixel_format = supported_image_encodings.at(cam->ImageFormat());

  // copy from simulation image to ROS msg
  fillImage(this->image_msg_, pixel_format, cam->ImageHeight(),
            cam->ImageWidth(), cam->ImageDepth() * cam->ImageWidth(),
            reinterpret_cast<const void *>(cam->ImageData()));

  // identify camera rendering
  std::map<std::string, rendering::CameraPtr> cameras = {
      {COLOR_CAMERA_NAME, this->colorCam},
  };
  if (this->ired1Cam) {
    cameras[IRED1_CAMERA_NAME] = this->ired1Cam;
  }
  if (this->ired2Cam) {
    cameras[IRED2_CAMERA_NAME] = this->ired2Cam;
  }

  // publish to ROS
  auto camera_info_msg =
      cameraInfo(this->image_msg_, cameras.at(camera_id)->HFOV().Radian());
  image_pub->publish(this->image_msg_, camera_info_msg);
}

void GazeboRosRealsense::InitPointCloudLUT(uint32_t rows, uint32_t cols, double hfov) {
  if (pcLutReady_) return;
  double fl = (double)cols / (2.0 * tan(hfov / 2.0));
  tanX_.resize(cols);
  tanY_.resize(rows);
  double halfCols = 0.5 * (double)(cols - 1);
  double halfRows = 0.5 * (double)(rows - 1);
  for (uint32_t i = 0; i < cols; i++)
    tanX_[i] = (float)tan(atan2((double)i - halfCols, fl));
  for (uint32_t j = 0; j < rows; j++)
    tanY_[j] = (float)tan(atan2((double)j - halfRows, fl));

  // Pre-allocate point cloud message once
  sensor_msgs::PointCloud2Modifier pcd_modifier(pointcloud_msg_);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows * cols);

  pcLutReady_ = true;
  std::cout << "RealSensePlugin: Point cloud LUT initialized (" << cols << "x" << rows << ")" << std::endl;
}

bool GazeboRosRealsense::FillPointCloudHelper(sensor_msgs::PointCloud2 &point_cloud_msg,
                                              uint32_t rows_arg, uint32_t cols_arg,
                                              uint32_t step_arg, void *data_arg)
{
  InitPointCloudLUT(rows_arg, cols_arg, this->depthCam->HFOV().Radian());

  point_cloud_msg.is_dense = true;
  const float *depthSrc = (const float *)data_arg;
  uint8_t *pcd_data = point_cloud_msg.data.data();
  const uint32_t point_step = point_cloud_msg.point_step;
  const float nan_val = std::numeric_limits<float>::quiet_NaN();

  // Check if color image matches depth resolution
  const bool hasColor = (this->image_msg_.data.size() == rows_arg * cols_arg * 3);
  const uint8_t *imgSrc = hasColor ? this->image_msg_.data.data() : nullptr;
  const bool isRGB = hasColor && (this->image_msg_.encoding == sensor_msgs::image_encodings::RGB8);

  uint32_t idx = 0;
  for (uint32_t j = 0; j < rows_arg; j++) {
    const float ty = tanY_[j];
    for (uint32_t i = 0; i < cols_arg; i++, idx++) {
      float *xyz = reinterpret_cast<float*>(pcd_data + idx * point_step);
      uint8_t *rgb = pcd_data + idx * point_step + 12;  // offset after xyz (3 floats = 12 bytes)

      const float depth = depthSrc[idx];
      if (depth > pointCloudCutOff_ && depth < pointCloudCutOffMax_) {
        xyz[0] = depth * tanX_[i];
        xyz[1] = depth * ty;
        xyz[2] = depth;
      } else {
        xyz[0] = xyz[1] = xyz[2] = nan_val;
        point_cloud_msg.is_dense = false;
      }

      if (hasColor) {
        const uint32_t px = (i + j * cols_arg) * 3;
        if (isRGB) {
          rgb[2] = imgSrc[px + 0];
          rgb[1] = imgSrc[px + 1];
          rgb[0] = imgSrc[px + 2];
        } else {
          rgb[0] = imgSrc[px + 0];
          rgb[1] = imgSrc[px + 1];
          rgb[2] = imgSrc[px + 2];
        }
      } else {
        rgb[0] = rgb[1] = rgb[2] = 0;
      }
    }
  }

  point_cloud_msg.height = rows_arg;
  point_cloud_msg.width = cols_arg;
  point_cloud_msg.row_step = point_step * cols_arg;
  return true;
}

void GazeboRosRealsense::OnNewDepthFrame() {
  // get current time
  common::Time current_time = this->world->SimTime();

  RealSensePlugin::OnNewDepthFrame();

  // copy data into image
  this->depth_msg_.header.frame_id =
      this->cameraParamsMap_[DEPTH_CAMERA_NAME].optical_frame;
  ;
  this->depth_msg_.header.stamp.sec = current_time.sec;
  this->depth_msg_.header.stamp.nsec = current_time.nsec;

  // set image encoding
  std::string pixel_format = sensor_msgs::image_encodings::TYPE_16UC1;

  // copy from simulation image to ROS msg
  fillImage(this->depth_msg_, pixel_format, this->depthCam->ImageHeight(),
            this->depthCam->ImageWidth(), 2 * this->depthCam->ImageWidth(),
            reinterpret_cast<const void *>(this->depthMap.data()));

  // publish to ROS
  auto depth_info_msg =
      cameraInfo(this->depth_msg_, this->depthCam->HFOV().Radian());
  this->depth_pub_.publish(this->depth_msg_, depth_info_msg);

  if (pointCloud_ && this->pointcloud_pub_.getNumSubscribers() > 0)
  {
    this->pointcloud_msg_.header = this->depth_msg_.header;
    this->pointcloud_msg_.width = this->depthCam->ImageWidth();
    this->pointcloud_msg_.height = this->depthCam->ImageHeight();
    this->pointcloud_msg_.row_step =
        this->pointcloud_msg_.point_step * this->depthCam->ImageWidth();
    // Use noisy depth data if distance noise is enabled, otherwise use raw
    void *depthSource = useDistanceNoise_
        ? (void *)this->noisyDepthData.data()
        : (void *)this->depthCam->DepthData();
    FillPointCloudHelper(this->pointcloud_msg_, this->depthCam->ImageHeight(),
                         this->depthCam->ImageWidth(), 2 * this->depthCam->ImageWidth(),
                         depthSource);
    this->pointcloud_pub_.publish(this->pointcloud_msg_);
  }
}
}

namespace {
std::string extractCameraName(const std::string &name) {
  if (name.find(COLOR_CAMERA_NAME) != std::string::npos)
    return COLOR_CAMERA_NAME;
  // Check ired2 before ired1 since "ired1" contains "ired"
  if (name.find(IRED2_CAMERA_NAME) != std::string::npos)
    return IRED2_CAMERA_NAME;
  if (name.find(IRED1_CAMERA_NAME) != std::string::npos)
    return IRED1_CAMERA_NAME;

  ROS_ERROR("Unknown camera name: %s", name.c_str());
  return COLOR_CAMERA_NAME;
}

sensor_msgs::CameraInfo cameraInfo(const sensor_msgs::Image &image,
                                   float horizontal_fov) {
  sensor_msgs::CameraInfo info_msg;

  info_msg.header = image.header;
  info_msg.distortion_model = "plumb_bob";
  info_msg.height = image.height;
  info_msg.width = image.width;

  float focal = 0.5 * image.width / tan(0.5 * horizontal_fov);

  info_msg.K[0] = focal;
  info_msg.K[4] = focal;
  info_msg.K[2] = info_msg.width * 0.5;
  info_msg.K[5] = info_msg.height * 0.5;
  info_msg.K[8] = 1.;

  info_msg.P[0] = info_msg.K[0];
  info_msg.P[5] = info_msg.K[4];
  info_msg.P[2] = info_msg.K[2];
  info_msg.P[6] = info_msg.K[5];
  info_msg.P[10] = info_msg.K[8];

  //    info_msg.roi.do_rectify = true;

  return info_msg;
}
}
