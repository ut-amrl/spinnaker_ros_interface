camera_serial = "19423356";

camera_img_width = 612;
camera_img_height = 512;
camera_img_fmt = "BayerRG8";
camera_exposure = 100.0; -- 20000 microseconds.
camera_exposure_auto = "Continuous";
camera_gain_auto = "Continuous";
camera_gamma = 0.65;

camera_enable_isp = true;

camera_enable_decimation = false;
camera_decimation = 1;
camera_enable_binning = true;
camera_binning = 4;

camera_line_selector = "Line0";
camera_line_mode = "Input";
camera_enable_3v3 = true;

ros_image_encoding = "bayer_rggb8";
ros_image_topic = "right";
ros_pub_camera_info = true;
