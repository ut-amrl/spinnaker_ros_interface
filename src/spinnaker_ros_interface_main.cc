// Copyright (c) 2020 Joydeep Biswas, joydeepb@cs.utexas.edu
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>

#include <iostream>

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "config_reader/config_reader.h"

DECLARE_int32(v);

DEFINE_bool(list, false, "List cameras");

DEFINE_string(config, "config/blackfly-s.lua", "Config file to load");

CONFIG_STRING(serial, "serial");
CONFIG_STRING(topic, "image_topic");

ros::Publisher image_pub_;

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
      printf("%lu Serial:%s Type:%s\n", i, 
          serial->ToString().c_str(), model->ToString().c_str());
      printf("\n");
    }
    cam_list.Clear();
  } catch(int e) {
    fprintf(stderr, "Unknown exception!\n");
  }
}

CameraPtr OpenCamera(CameraList& cam_list) {
  SystemPtr system = System::GetInstance();
  
  CHECK_GT(cam_list.GetSize(), 0) << "\nNo cameras found, quitting.";
  if (CONFIG_serial.empty()) {
    printf("Using first camera\n");
    return cam_list.GetByIndex(0);
  } else {
    printf("Get camera with serial %s\n", CONFIG_serial.c_str());
    return cam_list.GetBySerial(CONFIG_serial);
  }
}

void ConfigureCamera(Spinnaker::CameraPtr camera) {
  // Retrieve GenICam nodemap
  Spinnaker::GenApi::INodeMap& nodeMap = camera->GetNodeMap();
  try {
    CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
        // Retrieve the desired entry node from the enumeration node
        CEnumEntryPtr ptrPixelFormatMono8 = 
            ptrPixelFormat->GetEntryByName("BayerRG8");
        // CEnumEntryPtr ptrPixelFormatMono8 = ptrPixelFormat->GetEntryByName("Mono8");
        if (IsAvailable(ptrPixelFormatMono8) && IsReadable(ptrPixelFormatMono8)) {
            // Retrieve the integer value from the entry node
            int64_t pixelFormatMono8 = ptrPixelFormatMono8->GetValue();
            // Set integer as new value for enumeration node
            ptrPixelFormat->SetIntValue(pixelFormatMono8);
        } else {
            cout << "Pixel format mono 8 not available..." << endl;
        }
    } else {
        cout << "Pixel format not available..." << endl;
    }
    CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
    if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX)) {
        ptrOffsetX->SetValue(ptrOffsetX->GetMin());
        cout << "Offset X set to " << ptrOffsetX->GetMin() << "..." << endl;
    } else {
        cout << "Offset X not available..." << endl;
    }
    CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
    if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY)) {
        ptrOffsetY->SetValue(ptrOffsetY->GetMin());
        cout << "Offset Y set to " << ptrOffsetY->GetValue() << "..." << endl;
    } else {
        cout << "Offset Y not available..." << endl;
    }
    CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
    if (IsAvailable(ptrWidth) && IsWritable(ptrWidth)) {
        int64_t widthToSet = ptrWidth->GetMax();
        ptrWidth->SetValue(widthToSet);
        cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
    } else {
        cout << "Width not available..." << endl;
    }
    CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
    if (IsAvailable(ptrHeight) && IsWritable(ptrHeight)) {
        int64_t heightToSet = ptrHeight->GetMax();
        ptrHeight->SetValue(heightToSet);
        cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
    } else {
        cout << "Height not available..." << endl << endl;
    }
  } catch (Spinnaker::Exception& e) {
      LOG(FATAL) << "Spinnaker Error: " << e.what();
  }
}

void CaptureLoop(CameraPtr pCam) {
  Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();
  try {
    // Set acquisition mode to continuous
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    CHECK(IsAvailable(ptrAcquisitionMode)) << "Acquisition unavailable";
    CHECK(IsWritable(ptrAcquisitionMode)) << "Unable to set acquisition mode";
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
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
    CONFIG_STRING(frame_id, "frame_id");

    image.header.frame_id = CONFIG_frame_id;
    while (ros::ok()) {
      ImagePtr pResultImage = pCam->GetNextImage(1000);
      if (pResultImage->IsIncomplete()) {
        std::cerr << "Image incomplete with image status " 
                  << pResultImage->GetImageStatus() 
                  << endl;
      } else {
        image.header.stamp.fromSec(
              1e-9 * static_cast<double>(pResultImage->GetTimeStamp()));
        image.width = pResultImage->GetWidth();
        image.height = pResultImage->GetHeight();
        image.step = pResultImage->GetStride();
        image.data.resize(pResultImage->GetBufferSize());
        image.encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
        memcpy(image.data.data(), 
            pResultImage->GetData(), image.data.size());
        image_pub_.publish(image);
        if (FLAGS_v > 0) {
          printf("Image captured, t=%f\n", image.header.stamp.toSec());
        }
      }

      // Release image
      pResultImage->Release();
    }

    // End acquisition
    pCam->EndAcquisition();
  } catch (Spinnaker::Exception& e) {
    LOG(FATAL) << "Spinnaker Error: " << e.what() << endl;
  }
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
  ros::NodeHandle n;
  image_pub_ = n.advertise<sensor_msgs::Image>(CONFIG_topic, 1, false);
  SystemPtr system = System::GetInstance();
  CameraList cam_list = system->GetCameras();
  CameraPtr camera = OpenCamera(cam_list);
  camera->Init();
  ConfigureCamera(camera);
  CaptureLoop(camera);
  return 0;
}
