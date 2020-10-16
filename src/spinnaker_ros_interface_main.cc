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
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "config_reader/config_reader.h"

DEFINE_bool(list, false, "List cameras");

DEFINE_string(config, "config/blackfly-s.lua", "Config file to load");

CONFIG_STRING(serial, "serial");
CONFIG_STRING(topic, "topic");

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
    CameraList camList = system->GetCameras();
    printf("%d cameras found.\n", camList.GetSize());
    for (size_t i = 0; i < camList.GetSize(); ++i) {
      printf("%d Serial:%s", i, camList[i]->DeviceSerialNumber().c_str());
      camList[i]->DeviceFamilyName();
      printf("\n");
    }
  } catch(int e) {
    fprintf(stderr, "Unknown exception!\n");
  }
}

CameraPtr OpenCamera() {
  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();
  CHECK_GT(camList.GetSize(), 0);
  if (CONFIG_serial.empty()) {
    return camList[0];
  } else {
    return camList.GetBySerial(CONFIG_serial);
  }
}

void ConfigureCamera(Spinnaker::CameraPtr camera) {
  // Retrieve GenICam nodemap
  Spinnaker::GenApi::INodeMap& nodeMap = camera->GetNodeMap();
  int result = 0;
  try {
    CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
    if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat)) {
        // Retrieve the desired entry node from the enumeration node
        CEnumEntryPtr ptrPixelFormatMono8 = ptrPixelFormat->GetEntryByName("Mono8");
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
      cout << "Error: " << e.what() << endl;
      result = -1;
  }
}

void CaptureLoop(CameraPtr pCam) {
  cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
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
          memcpy(image.data.data(), 
              pResultImage->GetData(), image.data.size());
          image_pub_.publish(image);
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
  CameraPtr camera = OpenCamera();
  camera->Init();
  ConfigureCamera(camera);
  CaptureLoop(camera);

  return 0;
}
