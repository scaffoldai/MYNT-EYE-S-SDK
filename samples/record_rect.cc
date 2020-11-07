// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mynteye/logger.h"
#include "mynteye/device/device.h"
#include "mynteye/device/utils.h"
#include "mynteye/util/times.h"

#include "dataset.h"

MYNTEYE_USE_NAMESPACE

cv::Mat grab_frame(std::shared_ptr<API> api) {
  api->WaitForStreams();
  auto &&left_data = api->GetStreamData(Stream::LEFT_RECTIFIED);
  auto &&right_data = api->GetStreamData(Stream::RIGHT_RECTIFIED);

  if (!left_data.frame.empty() && !right_data.frame.empty()) {
    //cv::Mat img;
    //cv::hconcat(left_data.frame, right_data.frame, img);
    //cv::imshow(frame_name, img);
    return left_data.frame;
  }
  cv::Mat empty;
  return empty;
}

void init_stream(std::shared_ptr<API> api) {
  //auto request = api->GetStreamRequest();
  //request.fps = 10;
  //api->ConfigStreamRequest(request);
  api->EnableMotionDatas();
  api->EnableStreamData(Stream::LEFT_RECTIFIED);
  api->EnableStreamData(Stream::RIGHT_RECTIFIED);
  api->Start(Source::ALL);
}

int main(int argc, char *argv[]) {
  std::string out_dir = "/media/pi/localdata/mynteye";
  std::string viz_device = "00D325030009072C"; // Inna's MYNT eye
  int device_idx;
  int showviz = 0;
  
  glog_init _(argc, argv);
  if (argc >= 2) {
    device_idx = std::stoi(argv[1]);
  }
  else {
    std::cout << "Device index not specified" << std::endl;
    return 1;
  }
  
  std::size_t num_devices = device::get_num_devices();
  std::cout << "There are " << num_devices << " devices" << std::endl;
  
  std::shared_ptr<API> &&api = API::Create(argc, argv, device_idx);

  if (api != nullptr) {
    std::cout << "Successfully opened device with serial number " << api->GetInfo()->serial_number << std::endl;
    if (viz_device.compare(api->GetInfo()->serial_number) == 0) {
      showviz = 1;
    }
    init_stream(api);
  }
  else {
    return 1;
  }
  
  if (showviz) {
    cv::namedWindow("frame");
  }
  
  while (true) {
    cv::Mat img = grab_frame(api);
    
    if (showviz) {
      if (!img.empty()) {
          cv::imshow("frame", img);
      }
      
      char key = static_cast<char>(cv::waitKey(1));
      if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
        break;
      }
    }
  }
  
  return 0;
}