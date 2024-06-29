#include "miivii_gmsl_camera/miivii_gmsl_camera.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono;
using std::string;

namespace miivii_gmsl
{

  MiiviiGmslCamera::MiiviiGmslCamera(rclcpp::NodeOptions const &options) : rclcpp::Node{"miivii_gmsl_camera", options}
  {
    this->active_camera_num = 0;
    this->last_time = rclcpp::Time(0);
    DeclareParameters();
    GetParameters();
    CreateMvGmslCamera();
    // StartTimer();
  }

  MiiviiGmslCamera::~MiiviiGmslCamera()
  {
  }

  void MiiviiGmslCamera::SpiltResolution(std::string camera_resolution, uint *width, uint *height)
  {
    std::size_t found = camera_resolution.find("x");

    if (found != std::string::npos)
    {
      std::istringstream iss(camera_resolution);
      std::vector<std::string> parts;
      std::string part;

      while (std::getline(iss, part, 'x'))
      {
        parts.push_back(part);
      }

      *width = std::stoul(parts[0]);
      *height = std::stoul(parts[1]);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid Camera resolution format.");
      rclcpp::shutdown();
    }
  }

  void MiiviiGmslCamera::DeclareParameters()
  {
    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<int>("sync_camera_number", 8);
    declare_parameter<int>("async_camera_number", 0);
    declare_parameter<int>("sync_freq", 30);
    declare_parameter<int>("async_freq", 0);
    declare_parameter<int>("sync_trigger", 0xff);
    declare_parameter<int>("async_trigger", 0);
    declare_parameter<std::vector<int64_t>>("async_angle", std::vector<int64_t>());

    this->camera_num = 8;
    std::map<std::string, rclcpp::ParameterValue> parameters[8];
    for (int i = 0; i < this->camera_num; i++)
    {
      std::string dev_node = "video" + std::to_string(i);
      parameters[i] = {
          {dev_node + ".node_name", rclcpp::ParameterValue("/dev/" + dev_node)},
          {dev_node + ".active", rclcpp::ParameterValue(false)},
          {dev_node + ".camera_fmt", rclcpp::ParameterValue(DEFAULT_CAMERA_FORMAT)},
          {dev_node + ".camera_res", rclcpp::ParameterValue(DEFAULR_CAMERA_RESOLUTION)},
          {dev_node + ".output_fmt", rclcpp::ParameterValue(DEFAULT_OUTPUT_FORMAT)},
          {dev_node + ".output_res", rclcpp::ParameterValue(DEFAULR_OUTPUT_RESOLUTION)},
          {dev_node + ".params_file", rclcpp::ParameterValue("")}};
      for (const auto &kv : parameters[i])
      {
        this->declare_parameter(kv.first, kv.second);
      }
    }
  }

  void MiiviiGmslCamera::GetParameters()
  {
    get_parameter("publish_rate", this->publish_rate);
    get_parameter("sync_camera_number", this->sync_camera_num);
    get_parameter("async_camera_number", this->async_camera_num);
    get_parameter("sync_freq", this->sync_camera_freq);
    get_parameter("async_freq", this->async_camera_freq);
    get_parameter("sync_trigger", this->sync_camera_trigger);
    get_parameter("async_trigger", this->async_camera_trigger);

    get_parameter("async_angle", this->async_angle);

    RCLCPP_INFO(this->get_logger(), "Publish Rate(publish_rate) is: %f", this->publish_rate);
    RCLCPP_INFO(this->get_logger(), "Sync Camera Number(sync_camera_number) is: %d", this->sync_camera_num);
    RCLCPP_INFO(this->get_logger(), "ASync Camera Number(async_camera_number) is: %d", this->async_camera_num);
    RCLCPP_INFO(this->get_logger(), "Sync Freq(sync_freq) : %d", this->sync_camera_freq);
    RCLCPP_INFO(this->get_logger(), "ASync Freq(async_freq) : %d", this->async_camera_freq);
    RCLCPP_INFO(this->get_logger(), "Sync Trigger(sync_trigger) : %d", this->sync_camera_trigger);
    RCLCPP_INFO(this->get_logger(), "ASync Trigger(async_trigger) : %d", this->async_camera_trigger);
  }

  bool MiiviiGmslCamera::fileExists(const std::string &filename)
  {
    std::ifstream file(filename);
    return file.good();
  }

  sensor_msgs::msg::CameraInfo MiiviiGmslCamera::createCameraInfoFromYAML(const std::string &filepath)
  {
    if (filepath.empty() || !fileExists(filepath))
    {
      RCLCPP_INFO(this->get_logger(), "Params file is empty or not exist!");
      return sensor_msgs::msg::CameraInfo();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Params file is : %s", filepath.c_str());
    }

    sensor_msgs::msg::CameraInfo camera_info;
    try
    {
      YAML::Node config = YAML::LoadFile(filepath);

      camera_info.height = config["image_height"].as<int>();
      camera_info.width = config["image_width"].as<int>();
      // camera_info.header.frame_id = config["camera_name"].as<std::string>();
      camera_info.distortion_model = config["distortion_model"].as<std::string>();

      // 读取相机矩阵
      auto camera_matrix_data = config["camera_matrix"]["data"].as<std::vector<double>>();
      std::copy(camera_matrix_data.begin(), camera_matrix_data.end(), camera_info.k.begin());

      // 读取畸变系数
      auto distortion_coeffs = config["distortion_coefficients"]["data"].as<std::vector<double>>();
      camera_info.d = distortion_coeffs;

      // 读取投影矩阵
      auto projection_matrix_data = config["projection_matrix"]["data"].as<std::vector<double>>();
      std::copy(projection_matrix_data.begin(), projection_matrix_data.end(), camera_info.p.begin());

      // 读取矫正矩阵
      auto rectification_matrix_data = config["rectification_matrix"]["data"].as<std::vector<double>>();
      std::copy(rectification_matrix_data.begin(), rectification_matrix_data.end(), camera_info.r.begin());
    }
    catch (const YAML::Exception &e)
    {
      RCLCPP_INFO(this->get_logger(), "YAML Exception! %s", e.what());
    }

    return camera_info;
  }

  void MiiviiGmslCamera::CreateCtx()
  {
    for (int i = 0; i < camera_num; i++)
    {
      std::string dev_node = "video" + std::to_string(i);
      bool active = get_parameter(dev_node + ".active").as_bool();
      if (active)
      {
        image_publisher_[active_camera_num] = this->create_publisher<sensor_msgs::msg::Image>("miivii_gmsl/image" + std::to_string(i), 10);
        std::string camera_info_topic = "miivii_gmsl/camera_info" + std::to_string(i);
        camera_info_publisher_[active_camera_num] = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);
        ctx[active_camera_num].dev_node = get_parameter(dev_node + ".node_name").as_string();
        ctx[active_camera_num].camera_fmt_str = get_parameter(dev_node + ".camera_fmt").as_string();
        ctx[active_camera_num].output_fmt_str = get_parameter(dev_node + ".output_fmt").as_string();
        std::string camera_resolution = get_parameter(dev_node + ".camera_res").as_string();
        std::string output_resolution = get_parameter(dev_node + ".output_res").as_string();
        uint cam_w, cam_h, out_w, out_h;
        SpiltResolution(camera_resolution, &(cam_w), &(cam_h));
        SpiltResolution(output_resolution, &(out_w), &(out_h));
        ctx[active_camera_num].cam_w = cam_w;
        ctx[active_camera_num].cam_h = cam_h;
        ctx[active_camera_num].out_w = out_w;
        ctx[active_camera_num].out_h = out_h;

        std::string params_file_name = get_parameter(dev_node + ".params_file").as_string();

        RCLCPP_INFO(this->get_logger(), "===============================%s=====================", dev_node.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera Format(camera_fmt) is : %s", ctx[active_camera_num].camera_fmt_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Camera Resolution(camera_resolution) is : %s", camera_resolution.c_str());
        RCLCPP_INFO(this->get_logger(), "Output Format(output_fmt) is: %s", ctx[active_camera_num].output_fmt_str.c_str());
        RCLCPP_INFO(this->get_logger(), "Output Resolution(output_resolution) is : %s", output_resolution.c_str());

        camera_info_msg_.push_back(createCameraInfoFromYAML(params_file_name));

        this->active_camera_num++;
      }
    }
  }

  void MiiviiGmslCamera::CreateMvGmslCamera()
  {
    stCameraCfgSend.async_camera_num = this->async_camera_num;
    stCameraCfgSend.async_freq = this->async_camera_freq;
    stCameraCfgSend.async_camera_bit_draw = this->async_camera_trigger;
    stCameraCfgSend.sync_camera_num = this->sync_camera_num;
    stCameraCfgSend.sync_freq = this->sync_camera_freq;
    stCameraCfgSend.sync_camera_bit_draw = this->sync_camera_trigger;

    int async_angle_size = async_angle.size();
    if (async_angle_size == 8)
    {
      std::vector<uint>::iterator it;
      for (int i = 0; i < async_angle_size; i++)
      {
        stCameraCfgSend.async_camera_pos[i] = async_angle.at(i);
      }
    }

    CreateCtx();

    if (this->active_camera_num == 0)
    {
      this->set_parameter(rclcpp::Parameter("video0.active", true));
      CreateCtx();
    }

    mvcam = new miivii::MvGmslCamera(ctx, this->active_camera_num, stCameraCfgSend);

    time_publisher_ = this->create_publisher<miivii_gmsl_camera::msg::MiiviiTime>("miivii_gmsl/time", 10);
  }

  void MiiviiGmslCamera::StartTimer()
  {
    // Set Timer default to 33ms
    if (std::abs(this->publish_rate) < std::numeric_limits<double>::epsilon())
    {
      RCLCPP_WARN(get_logger(), "Invalid publish_rate = 0. Use default value 20 instead");
      this->publish_rate = 30.0;
    }
    if (this->publish_rate > 0)
    {
      const auto publish_period = rclcpp::Rate(this->publish_rate).period();
      image_publisher_timer_ = this->create_wall_timer(publish_period, std::bind(&MiiviiGmslCamera::timer_callback, this));
    }
  }
  void MiiviiGmslCamera::timer_callback()
  {
    uint8_t *outbuf[this->active_camera_num];
    uint64_t timestamp;
    uint8_t camera_no = ctx[this->active_camera_num - 1].dev_node[10] - 0x30;

    bool res = this->mvcam->GetImagePtr(outbuf, timestamp, camera_no, this->g_camera_dev);

    if (!res)
    {
      RCLCPP_ERROR(this->get_logger(), "Get Image Error!!");
    }

    rclcpp::Clock clock;
    rclcpp::Time now = clock.now();
    rclcpp::Time time(timestamp);

    if (this->last_time == rclcpp::Time(0))
    {
      this->last_time = time;
    }
    else
    {
      rclcpp::Duration latency = now - time;
      rclcpp::Duration jitter = time - this->last_time;
      this->last_time = time;

      builtin_interfaces::msg::Duration converted_jitter_duration;
      converted_jitter_duration.sec = static_cast<int32_t>(jitter.nanoseconds() / 1000000000LL);
      converted_jitter_duration.nanosec = static_cast<uint32_t>(jitter.nanoseconds() % 1000000000LL);

      builtin_interfaces::msg::Duration converted_latency_duration;
      converted_latency_duration.sec = static_cast<int32_t>(latency.nanoseconds() / 1000000000LL);
      converted_latency_duration.nanosec = static_cast<uint32_t>(latency.nanoseconds() % 1000000000LL);

      // uint64_t time_interval{};
      // time_interval = 1000000000 / stCameraCfgSend.sync_freq;
      // if (static_cast<uint64_t>(jitter.nanoseconds()) > (time_interval + 5000000) || static_cast<uint64_t>(jitter.nanoseconds()) < (time_interval - 5000000))
      // {
      //   RCLCPP_INFO(this->get_logger(), "jitter : %ld", jitter.nanoseconds());
      // }

      miivii_gmsl_camera::msg::MiiviiTime miiviimsg;
      miiviimsg.header.stamp = time;
      std::vector<std::string> nodes;
      std::vector<builtin_interfaces::msg::Duration> latencys;
      std::vector<builtin_interfaces::msg::Duration> jitters;
      for (int i = 0; i < this->active_camera_num; i++)
      {
        nodes.push_back(ctx[i].dev_node);
        latencys.push_back(converted_latency_duration);
        jitters.push_back(converted_jitter_duration);
      }
      miiviimsg.nodes = nodes;
      miiviimsg.jitter = jitters;
      miiviimsg.latency = latencys;
      time_publisher_->publish(miiviimsg);
    }

    for (int i = 0; i < this->active_camera_num; i++)
    {
      std::string encoding;
      int step_factor;
      if (ctx[i].camera_fmt_str == "UYVY" && ctx[i].output_fmt_str == "UYVY")
      {
        encoding = "yuv422";
        step_factor = 2;
      }
      else if (ctx[i].camera_fmt_str == "UYVY" && ctx[i].output_fmt_str == "BGRA32")
      {
        encoding = "bgra8";
        step_factor = 4;
      }

      sensor_msgs::msg::Image image;

      image.header.stamp = time;
      image.header.frame_id = "camera" + std::to_string(ctx[i].dev_node[10] - 0x30);
      image.height = ctx[i].out_h;
      image.width = ctx[i].out_w;
      image.encoding = encoding;
      image.is_bigendian = false;
      image.step = step_factor * image.width;
      std::vector<uint8_t> image_data(outbuf[i], outbuf[i] + image.height * image.step);
      image.data = std::move(image_data);

      image_publisher_[i]->publish(image);

      sensor_msgs::msg::CameraInfo camera_info = camera_info_msg_.at(i);
      camera_info.header.stamp = image.header.stamp;
      camera_info.header.frame_id = image.header.frame_id;
      camera_info.height = ctx[i].out_h;
      camera_info.width = ctx[i].out_w;

      camera_info_publisher_[i]
          ->publish(camera_info);
    }
  }

}

// RCLCPP_COMPONENTS_REGISTER_NODE(miivii_gmsl::MiiviiGmslCamera)
