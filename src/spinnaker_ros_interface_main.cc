// Copyright (c) 2020 Joydeep Biswas, joydeepb@cs.utexas.edu
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>

#include <iostream>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"
#include "config_reader/config_reader.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/SetCameraInfo.h"
#include "sensor_msgs/image_encodings.h"

DECLARE_int32(v);

DEFINE_bool(list, false, "List cameras");

DEFINE_string(config, "config/blackfly-s.lua", "Config file to load");
DEFINE_bool(debayer, true, "Enable software debayering");

CONFIG_STRING(serial, "camera_serial");
CONFIG_INT(img_width, "camera_img_width");
CONFIG_INT(img_height, "camera_img_height");
CONFIG_STRING(img_fmt, "camera_img_fmt");
CONFIG_FLOAT(exposure, "camera_exposure");
CONFIG_STRING(exposure_auto, "camera_exposure_auto");
CONFIG_STRING(gain_auto, "camera_gain_auto");
CONFIG_INT(binning, "camera_binning");
CONFIG_INT(decimation, "camera_decimation");
CONFIG_BOOL(enable_isp, "camera_enable_isp");
CONFIG_BOOL(enable_binning, "camera_enable_binning");
CONFIG_BOOL(enable_decimation, "camera_enable_decimation");
CONFIG_FLOAT(gamma, "camera_gamma");

CONFIG_STRING(line_selector, "camera_line_selector");
CONFIG_STRING(line_mode, "camera_line_mode");
CONFIG_BOOL(enable3v3, "camera_enable_3v3");

CONFIG_STRING(topic, "ros_image_topic");
CONFIG_STRING(ros_image_encoding, "ros_image_encoding");
CONFIG_BOOL(ros_pub_camera_info, "ros_pub_camera_info");

image_transport::Publisher image_pub_;
ros::Publisher camera_info_pub_;
sensor_msgs::CameraInfo camera_info_;
ros::ServiceServer set_camera_info_srv_;

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

using std::cout;
using std::endl;

void EnumerateCameras() {
  try {
    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();
    // Retrieve list of cameras from the system
    CameraList cam_list = system->GetCameras();
    printf("%d cameras found.\n", cam_list.GetSize());
    for (size_t i = 0; i < cam_list.GetSize(); ++i) {
      CameraPtr camera = cam_list.GetByIndex(i);
      INodeMap& node_map = camera->GetTLDeviceNodeMap();
      CStringPtr serial = node_map.GetNode("DeviceSerialNumber");
      CStringPtr model = node_map.GetNode("DeviceModelName");
      printf("%lu Serial:%s Type:%s\n",
             i,
             serial->ToString().c_str(),
             model->ToString().c_str());
      printf("\n");
    }
    cam_list.Clear();
  } catch (int e) {
    fprintf(stderr, "Unknown exception!\n");
  }
}

CameraPtr OpenCamera(CameraList& cam_list) {
  SystemPtr system = System::GetInstance();

  CHECK_GT(cam_list.GetSize(), 0) << "\nNo cameras found, quitting.";
  if (CONFIG_serial.empty()) {
    std::cout << "Using first camera " << std::endl;
    printf("Using first camera\n");
    return cam_list.GetByIndex(0);
  } else {
    std::cout << "Get first camera" << std::endl;
    printf("Get camera with serial %s\n", CONFIG_serial.c_str());
    return cam_list.GetBySerial(CONFIG_serial);
  }
}

template <typename SettingType, typename ValueType>
void WriteSetting(const std::string& setting,
                  const ValueType& value,
                  Spinnaker::GenApi::INodeMap& nodeMap) {
  Spinnaker::GenApi::CPointer<SettingType> setting_node =
      nodeMap.GetNode(setting.c_str());
  if (!Spinnaker::GenApi::IsAvailable(setting_node)) {
    cout << "Setting " << setting << " not available\n";
    return;
  }
  if (!Spinnaker::GenApi::IsWritable(setting_node)) {
    cout << "Setting " << setting << " not writable\n";
    return;
  }
  setting_node->SetValue(value);
  cout << setting << " set to " << value << "\n";
}

template <typename SettingType, typename ValueType>
ValueType ReadSetting(const std::string& setting,
                      Spinnaker::GenApi::INodeMap& nodeMap) {
  Spinnaker::GenApi::CPointer<SettingType> setting_node =
      nodeMap.GetNode(setting.c_str());
  CHECK(Spinnaker::GenApi::IsAvailable(setting_node))
      << "\n"
      << setting << " not available\n";
  CHECK(Spinnaker::GenApi::IsReadable(setting_node))
      << "\n"
      << setting << " not readable\n";
  return setting_node->GetValue();
}

std::string ReadEnum(const std::string& setting,
                     Spinnaker::GenApi::INodeMap& nodeMap) {
  Spinnaker::GenApi::CEnumerationPtr setting_node =
      nodeMap.GetNode(setting.c_str());
  CHECK(Spinnaker::GenApi::IsAvailable(setting_node))
      << "\n"
      << setting << " not available!";
  CHECK(Spinnaker::GenApi::IsReadable(setting_node))
      << "\n"
      << setting << " not readable!";
  return setting_node->GetCurrentEntry()->GetSymbolic().c_str();
}

void SetEnum(const std::string& setting,
             const std::string& value,
             Spinnaker::GenApi::INodeMap& nodeMap) {
  Spinnaker::GenApi::CEnumerationPtr setting_node =
      nodeMap.GetNode(setting.c_str());
  CHECK(Spinnaker::GenApi::IsAvailable(setting_node))
      << "\n"
      << setting << " not available!";
  CHECK(Spinnaker::GenApi::IsReadable(setting_node))
      << "\n"
      << setting << " not readable!";
  CEnumEntryPtr enum_entry = setting_node->GetEntryByName(value.c_str());
  if (!IsAvailable(enum_entry)) {
    cout << "Enum entry \"" << value << "\" for setting \"" << setting
         << "\" Not available\n";
    return;
  }
  if (!IsReadable(enum_entry)) {
    cout << "Enum entry \"" << value << "\" for setting \"" << setting
         << "\" not readable\n";
    return;
  }
  setting_node->SetIntValue(enum_entry->GetValue());
  cout << setting << " set to " << value << "\n";
}

void ConfigureCamera(Spinnaker::CameraPtr camera) {
  // Retrieve GenICam nodemap
  Spinnaker::GenApi::INodeMap& nodeMap = camera->GetNodeMap();
  try {
    SetEnum("PixelFormat", CONFIG_img_fmt, nodeMap);
    SetEnum("ExposureAuto", CONFIG_exposure_auto, nodeMap);
    SetEnum("ExposureMode", "Timed", nodeMap);
    SetEnum("BinningSelector", "Sensor", nodeMap);

    /** Primary Camera Testing **/
    SetEnum("LineSelector", CONFIG_line_selector, nodeMap);
    SetEnum("LineMode", CONFIG_line_mode, nodeMap);

    WriteSetting<Spinnaker::GenApi::IBoolean, float>(
        "V3_3Enable", CONFIG_enable3v3, nodeMap);

    /** End Camera Testing **/
    WriteSetting<IInteger, int>("Width", CONFIG_img_width, nodeMap);
    WriteSetting<IInteger, int>("Height", CONFIG_img_height, nodeMap);
    SetEnum("BinningSelector", "All", nodeMap);
    // SetEnum("BinningVerticalMode", "Additive", nodeMap);
    WriteSetting<Spinnaker::GenApi::IBoolean, float>(
        "IspEnable", CONFIG_enable_isp, nodeMap);
    WriteSetting<Spinnaker::GenApi::IFloat, float>(
        "ExposureTime", CONFIG_exposure, nodeMap);
    WriteSetting<Spinnaker::GenApi::IFloat, float>(
        "Gamma", CONFIG_gamma, nodeMap);
    SetEnum("GainAuto", CONFIG_gain_auto, nodeMap);
    WriteSetting<Spinnaker::GenApi::IBoolean, float>(
        "AcquisitionFrameRateEnable", false, nodeMap);
    if (CONFIG_enable_decimation) {
      WriteSetting<Spinnaker::GenApi::IInteger, float>(
          "DecimationHorizontal", CONFIG_decimation, nodeMap);
      WriteSetting<Spinnaker::GenApi::IInteger, float>(
          "DecimationVertical", CONFIG_decimation, nodeMap);
    }
    if (CONFIG_enable_binning) {
      WriteSetting<Spinnaker::GenApi::IInteger, float>(
          "BinningHorizontal", CONFIG_binning, nodeMap);
      WriteSetting<Spinnaker::GenApi::IInteger, float>(
          "BinningVertical", CONFIG_binning, nodeMap);
    }
    // WriteSetting<Spinnaker::GenApi::IFloat, float>(
    //     "AcquisitionFrameRate", 30.0f, nodeMap);
    cout << "\n\nResulting frame rate: "
         << ReadSetting<Spinnaker::GenApi::IFloat, float>(
                "AcquisitionResultingFrameRate", nodeMap)
         << "\n";
    cout << "Frame rate: "
         << ReadSetting<Spinnaker::GenApi::IFloat, float>(
                "AcquisitionFrameRate", nodeMap)
         << "\n";
    cout << "BinningVertical: "
         << ReadSetting<Spinnaker::GenApi::IInteger, float>("BinningVertical",
                                                            nodeMap)
         << "\n";
    cout << "DeviceMaxThroughput: "
         << ReadSetting<IInteger, int>("DeviceMaxThroughput", nodeMap) << "\n";
    cout << "DeviceLinkSpeed: "
         << ReadSetting<IInteger, int>("DeviceLinkSpeed", nodeMap) << "\n";
    cout << "DeviceLinkThroughputLimit: "
         << ReadSetting<IInteger, int>("DeviceLinkThroughputLimit", nodeMap)
         << "\n";
    cout << "DeviceLinkBandwidthReserve: "
         << ReadSetting<IFloat, float>("DeviceLinkBandwidthReserve", nodeMap)
         << "\n";
    cout << "LineStatus: "
         << ReadSetting<IBoolean, float>("LineStatus", nodeMap) << "\n";
    cout << "LineStatusAll: "
         << ReadSetting<IInteger, int>("LineStatusAll", nodeMap) << "\n";
  } catch (Spinnaker::Exception& e) {
    LOG(FATAL) << "Spinnaker Error: " << e.what();
  }
}

void CaptureLoop(CameraPtr pCam) {
  Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();
  try {
    std::cout << "Starting acquisition\n";
    // Set acquisition mode to continuous
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    CHECK(IsAvailable(ptrAcquisitionMode)) << "Acquisition unavailable";
    CHECK(IsWritable(ptrAcquisitionMode)) << "Unable to set acquisition mode";
    CEnumEntryPtr ptrAcquisitionModeContinuous =
        ptrAcquisitionMode->GetEntryByName("Continuous");
    CHECK(IsAvailable(ptrAcquisitionModeContinuous))
        << "Continuous acquisition mode unavailable";
    CHECK(IsReadable(ptrAcquisitionModeContinuous))
        << "Continuous acquisition mode not readable";
    int64_t acquisitionModeContinuous =
        ptrAcquisitionModeContinuous->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    // Begin acquiring images
    pCam->BeginAcquisition();

    sensor_msgs::Image image;

    image.header.frame_id = CONFIG_topic + "_optical";
    image.width = CONFIG_img_width;
    image.height = CONFIG_img_height;
    if (FLAGS_debayer && CONFIG_img_fmt == "BayerRG8") {
      image.encoding = sensor_msgs::image_encodings::BGR8;
    } else {
      image.encoding = CONFIG_ros_image_encoding;
    }

    if (CONFIG_ros_pub_camera_info) {
      camera_info_.header.frame_id = CONFIG_topic + "_optical";
      camera_info_.width = CONFIG_img_width;
      camera_info_.height = CONFIG_img_height;

      // Binning is done on camera not in ROS so set to 1 by default
      camera_info_.binning_x = 1;
      camera_info_.binning_y = 1;
    }

    // The net_offset accounts for the FLIR clock being non-
    // zero when the node starts and the ROS wall time
    double net_time_offset{0.0};

    while (ros::ok()) {
      ImagePtr pResultImage = pCam->GetNextImage(1000);
      if (pResultImage->IsIncomplete()) {
        std::cerr << "Image incomplete with image status "
                  << pResultImage->GetImageStatus() << endl;
      } else {
        // Executes on the first frame only and gives us the net ROS/FLIR camera
        // clock offset
        if (net_time_offset == 0.0) {
          net_time_offset =
              ros::Time::now().toSec() -
              1e-9 * static_cast<double>(pResultImage->GetTimeStamp());
        }
        image.header.stamp.fromSec(
            net_time_offset +
            1e-9 * (static_cast<double>(pResultImage->GetTimeStamp())));

        if (CONFIG_ros_pub_camera_info) {
          camera_info_.header.stamp = image.header.stamp;
          // Spin to check for and execute set_camera_info_srv_ service requests
          ros::spinOnce();
        }

        if (FLAGS_debayer && CONFIG_img_fmt == "BayerRG8") {
          // ImagePtr color_image = processor.Convert(pResultImage,
          // PixelFormat_BGR8);
          ImagePtr color_image = pResultImage->Convert(
              PixelFormat_BGR8, Spinnaker::NEAREST_NEIGHBOR);
          image.step = color_image->GetStride();
          image.data.resize(std::min<uint64_t>(image.height * image.step,
                                               color_image->GetBufferSize()));
          memcpy(image.data.data(), color_image->GetData(), image.data.size());
        } else {
          image.step = pResultImage->GetStride();
          image.data.resize(std::min<uint64_t>(image.height * image.step,
                                               pResultImage->GetBufferSize()));
          memcpy(image.data.data(), pResultImage->GetData(), image.data.size());
        }

        image_pub_.publish(image);
        camera_info_pub_.publish(camera_info_);

        if (FLAGS_v > 0) {
          printf("%dx%d %lu Image captured, t=%f\n",
                 image.width,
                 image.height,
                 image.data.size(),
                 image.header.stamp.toSec());
        }
      }

      // Release image
      pResultImage->Release();
    }

    // End acquisition
    pCam->EndAcquisition();
  } catch (Spinnaker::Exception& e) {
    pCam->EndAcquisition();
    LOG(FATAL) << "Spinnaker Error: " << e.what() << endl;
  }
}

bool SetCameraInfoSrvCallback(sensor_msgs::SetCameraInfo::Request& request,
                              sensor_msgs::SetCameraInfo::Response& response) {
  camera_info_.distortion_model = request.camera_info.distortion_model;
  camera_info_.D = request.camera_info.D;
  camera_info_.K = request.camera_info.K;
  camera_info_.R = request.camera_info.R;
  camera_info_.P = request.camera_info.P;

  response.success = true;
  response.status_message =
      "This service only allows you to set the distortion model and the DKRP "
      "matrices. You cannot change any other camera parameter!";

  return true;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  config_reader::ConfigReader config({FLAGS_config});
  if (FLAGS_list) {
    EnumerateCameras();
    return 0;
  }
  ros::init(argc, argv, "spinnaker_ros_interface");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_pub_ = it.advertise(CONFIG_topic + "/image_raw", 1, false);
  camera_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>(
      CONFIG_topic + "/camera_info", 1, false);
  set_camera_info_srv_ = nh.advertiseService(CONFIG_topic + "/set_camera_info",
                                             SetCameraInfoSrvCallback);
  std::cout << "Getting camera list\n";
  SystemPtr system = System::GetInstance();
  CameraList cam_list = system->GetCameras();
  CameraPtr camera = OpenCamera(cam_list);
  camera->Init();
  ConfigureCamera(camera);
  CaptureLoop(camera);

  return 0;
}
