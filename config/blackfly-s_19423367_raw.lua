camera_serial = "19423367";

camera_img_width = 612;
camera_img_height = 512;
camera_img_fmt = "BayerRG8"; -- BayerRG8
camera_exposure = 1000.0; -- 20000 microseconds.
camera_exposure_auto = "Continuous";
camera_gain_auto = "Continuous";
camera_gamma = 0.65;

camera_enable_isp = true;

camera_enable_decimation = false;
camera_decimation = 1;
camera_enable_binning = true;
camera_binning = 4;

camera_line_selector = "Line2";
camera_line_mode = "Input"; -- should be output but spinnake sdk doesn't recognize option
camera_enable_3v3 = true;

ros_image_encoding = "bayer_rggb8";
ros_image_topic = "left";
ros_pub_camera_info = true;