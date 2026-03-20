#include "vision_tune/process/process_node.hpp"
#include "vision_tune/utils/vision_utils.hpp"
#include "vision_tune/utils/common_utils.hpp"

static vision_tune::utils::hsv_range get_selected_hsv(const vision_tune::utils::hsv_config &cfg, vision_target target);

ProcessNode::ProcessNode()
    : Node("process_node")
{
  declare_parameters();
  get_parameters();

  if (!model_xml_.empty())
  {
    try
    {
      model_ = core_.read_model(model_xml_);
      compiled_model_ = core_.compile_model(model_, "AUTO");
      infer_request_ = compiled_model_.create_infer_request();
      model_loaded_ = true;
      RCLCPP_INFO(get_logger(), "YOLO model loaded: %s", model_xml_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_logger(), "YOLO model load failed: %s", e.what());
    }
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      1,
      std::bind(&ProcessNode::image_callback, this, std::placeholders::_1));

  tuning_sub_ = this->create_subscription<vision_tune::msg::TuningValue>(
      tuning_topic,
      1,
      std::bind(&ProcessNode::tuning_callback, this, std::placeholders::_1));

  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10,
      std::bind(&ProcessNode::camera_info_callback, this, std::placeholders::_1));

  pan_tilt_status_sub_ = this->create_subscription<vision_tune::msg::PanTiltStatusMsgs>(
      pan_tilt_status_topic_, 1,
      std::bind(&ProcessNode::pan_tilt_status_callback, this, std::placeholders::_1));

  result_pub_ = this->create_publisher<vision_tune::msg::ProcessResult>(
      result_topic, 1);

  result_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      result_image_topic, 1);

  bird_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      bird_image_topic, 1);

  timer_ = this->create_wall_timer(
      vision_tune::utils::hz_to_period(node_hz_), // yaml에서 토픽/주기 설정
      std::bind(&ProcessNode::process_tick, this));
}

void ProcessNode::declare_parameters() // ui_config.yaml
{
  declare_parameter<std::string>("topic.input_topic", "/camera1/camera/image_raw");
  declare_parameter<std::string>("topic.tuning_topic", "/vision/tuning");
  declare_parameter<std::string>("topic.result_topic", "/vision/result");
  declare_parameter<std::string>("topic.result_image_topic", "/vision/result_image");
  declare_parameter<std::string>("topic.bird_image_topic", "/vision/bird_image");
  declare_parameter<std::string>("camera_info_topic", "/camera1/info");
  declare_parameter<std::string>("pan_tilt_status_topic", "/camera1/pan_tilt_status");
  declare_parameter<double>("node_hz", 30.0);
  declare_parameter<double>("ROBOT_HEIGHT", 555.0);
  declare_parameter<std::string>("model_xml", "");
}

void ProcessNode::get_parameters()
{
  get_parameter("topic.input_topic", input_topic);
  get_parameter("topic.tuning_topic", tuning_topic);
  get_parameter("topic.result_topic", result_topic);
  get_parameter("topic.result_image_topic", result_image_topic);
  get_parameter("topic.bird_image_topic", bird_image_topic);
  get_parameter("camera_info_topic", camera_info_topic_);
  get_parameter("pan_tilt_status_topic", pan_tilt_status_topic_);
  get_parameter("node_hz", node_hz_);
  get_parameter("ROBOT_HEIGHT", cam_height_);
  get_parameter("model_xml", model_xml_);
}

void ProcessNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  cam_focal_.x     = msg->k[0];
  cam_focal_.y     = msg->k[4];
  cam_principal_.x = msg->k[2];
  cam_principal_.y = msg->k[5];
  camera_initialized_ = true;
}

void ProcessNode::pan_tilt_status_callback(const vision_tune::msg::PanTiltStatusMsgs::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  cam_tilt_ = static_cast<double>(msg->tilt);
}

void ProcessNode::tuning_callback(const vision_tune::msg::TuningValue::SharedPtr msg)
{ // ui에서 hsv값이 변경될 때마다 업데이트
  //RCLCPP_INFO(this->get_logger(), "tuning_callback called");
  std::lock_guard<std::mutex> lock(data_mutex_);

  hsv_config_.red.h_low = msg->red_h_low;
  hsv_config_.red.h_high = msg->red_h_high;
  hsv_config_.red.s_low = msg->red_s_low;
  hsv_config_.red.s_high = msg->red_s_high;
  hsv_config_.red.v_low = msg->red_v_low;
  hsv_config_.red.v_high = msg->red_v_high;

  hsv_config_.blue.h_low = msg->blue_h_low;
  hsv_config_.blue.h_high = msg->blue_h_high;
  hsv_config_.blue.s_low = msg->blue_s_low;
  hsv_config_.blue.s_high = msg->blue_s_high;
  hsv_config_.blue.v_low = msg->blue_v_low;
  hsv_config_.blue.v_high = msg->blue_v_high;

  hsv_config_.line.h_low = msg->line_h_low;
  hsv_config_.line.h_high = msg->line_h_high;
  hsv_config_.line.s_low = msg->line_s_low;
  hsv_config_.line.s_high = msg->line_s_high;
  hsv_config_.line.v_low = msg->line_v_low;
  hsv_config_.line.v_high = msg->line_v_high;

  selected_target_ = static_cast<vision_target>(msg->selected_target);
  tuning_dirty_ = true;
}

void ProcessNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{ // 버퍼에 저장만하고 실제 처리는 process_tick
  //RCLCPP_INFO(this->get_logger(), "image_callback called");
  try
  {
    cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_raw_mat_ = frame.clone();
    raw_dirty_ = true;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void ProcessNode::process_tick()
{
  // RCLCPP_INFO(this->get_logger(), "process_tick called");

  cv::Mat raw_mat;
  vision_tune::utils::hsv_config cfg;
  vision_target selected_target;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if ((!raw_dirty_ && !tuning_dirty_) || latest_raw_mat_.empty())
    {
      // RCLCPP_WARN(
      //     this->get_logger(),
      //     "process_tick return: raw_dirty=%d tuning_dirty=%d empty=%d",
      //     raw_dirty_,
      //     tuning_dirty_,
      //     latest_raw_mat_.empty());
      return;
    }

    raw_mat = latest_raw_mat_.clone();
    cfg = hsv_config_;
    selected_target = selected_target_;
    raw_dirty_ = false;
    tuning_dirty_ = false;
  }

  // ========================= bird view =========================
  std::vector<cv::Point2f> src_points = {
      {distort_L_top_x,   distort_L_top_y},
      {distort_R_top_x,   distort_R_top_y},
      {distort_L_under_x, distort_L_under_y},
      {distort_R_under_x, distort_R_under_y}};

  std::vector<cv::Point2f> dst_points = {
      {flat_L_top_x,   flat_L_top_y},
      {flat_R_top_x,   flat_R_top_y},
      {flat_L_under_x, flat_L_under_y},
      {flat_R_under_x, flat_R_under_y}};

  cv::Mat bird_view = vision_tune::utils::make_bird_view(raw_mat, src_points, dst_points, 320, 240);

  if (bird_view.empty())
  {
    RCLCPP_WARN(this->get_logger(), "bird_view is empty");
    return;
  }
  // ============================================================

  // ========================= selected target =========================
  vision_tune::utils::hsv_range selected_hsv =
      get_selected_hsv(cfg, selected_target);

  cv::Mat source_img = raw_mat;
  if (selected_target == vision_target::line)
  {
    source_img = bird_view;
  }

  cv::Mat mask = vision_tune::utils::make_hsv_mask(source_img, selected_hsv);

  if (mask.empty())
  {
    RCLCPP_WARN(this->get_logger(), "mask is empty");
    return;
  }

  auto detection = vision_tune::utils::find_largest_blob(mask);

  // result_image는 바이너리 마스크를 3채널로 바꾼 화면
  cv::Mat result_mat;
  cv::cvtColor(mask, result_mat, cv::COLOR_GRAY2BGR);

  vision_tune::msg::ProcessResult result_msg;
  result_msg.detected = detection.detected;
  result_msg.center_x = detection.center_x;
  result_msg.center_y = detection.center_y;
  result_msg.area = detection.area;

  // ========================= YOLO obstacle detection =========================
  auto obstacles = detect_obstacle(raw_mat);
  float best_score = 0.0f;
  for (const auto &obs : obstacles)
  {
    cv::rectangle(result_mat, obs.bbox, COLORS.at(0), 2);
    if (obs.score > best_score)
    {
      best_score = obs.score;
      result_msg.detected = true;
      result_msg.center_x = obs.center_x;
      result_msg.center_y = obs.center_y;
      result_msg.area     = obs.area;
      result_msg.score    = obs.score;
      result_msg.x1       = obs.bbox.x;
      result_msg.y1       = obs.bbox.y;
      result_msg.x2       = obs.bbox.x + obs.bbox.width;
      result_msg.y2       = obs.bbox.y + obs.bbox.height;
    }
  }
  // ===========================================================================

  // ========================= 거리 추정 =========================
  if (result_msg.detected && camera_initialized_)
  {
    cv::Point2d pixel_pt(result_msg.center_x, result_msg.center_y);
    auto pos = vision_tune::Cal_utils::calcObjectDistance(
        cam_tilt_, cam_height_, cam_focal_, cam_principal_, pixel_pt);
    result_msg.distance = pos.dist;
    result_msg.theta    = pos.theta;
  }
  // ==========================================================

  // ========================= publish =========================
  auto result_img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_mat).toImageMsg();

  auto bird_img_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bird_view).toImageMsg();

  result_img_msg->header.stamp = this->now();
  bird_img_msg->header.stamp = this->now();

  result_img_msg->header.frame_id = "result_frame";
  bird_img_msg->header.frame_id = "bird_frame";

  result_image_pub_->publish(*result_img_msg);
  bird_image_pub_->publish(*bird_img_msg);
  result_pub_->publish(result_msg);
  // ==========================================================
}

std::vector<vision_tune::utils::detection_result> ProcessNode::detect_obstacle(const cv::Mat &raw_mat)
{
  std::vector<vision_tune::utils::detection_result> results;
  if (!model_loaded_ || raw_mat.empty())
    return results;

  auto input_port = compiled_model_.input(0);
  auto input_shape = input_port.get_shape();
  int model_h = static_cast<int>(input_shape[2]);
  int model_w = static_cast<int>(input_shape[3]);
  int model_ch = static_cast<int>(input_shape[1]);
  int orig_h = raw_mat.rows;
  int orig_w = raw_mat.cols;

  // ---- 전처리 ----
  cv::Mat gray;
  cv::cvtColor(raw_mat, gray, cv::COLOR_BGR2GRAY);

  cv::Mat resized;
  if (model_ch == 1)
  {
    cv::resize(gray, resized, cv::Size(model_w, model_h));
  }
  else
  {
    cv::Mat gray3;
    cv::cvtColor(gray, gray3, cv::COLOR_GRAY2RGB);
    cv::resize(gray3, resized, cv::Size(model_w, model_h));
  }

  cv::Mat norm;
  resized.convertTo(norm, CV_32F, 1.0f / 255.0f);

  ov::Tensor input_tensor(input_port.get_element_type(), input_shape);
  float *input_data = input_tensor.data<float>();
  int ppc = model_h * model_w;

  if (model_ch == 1)
  {
    std::memcpy(input_data, norm.ptr<float>(), ppc * sizeof(float));
  }
  else
  {
    std::vector<cv::Mat> chs(model_ch);
    cv::split(norm, chs);
    for (int c = 0; c < model_ch; ++c)
      std::memcpy(input_data + c * ppc, chs[c].ptr<float>(), ppc * sizeof(float));
  }

  // ---- 추론 ----
  infer_request_.set_tensor(input_port, input_tensor);
  infer_request_.infer();

  // ---- 후처리 (class 0만) ----
  auto out = infer_request_.get_tensor(compiled_model_.output(0));
  const float *out_data = out.data<float>();
  size_t n_det = out.get_shape()[1];

  for (size_t i = 0; i < n_det; i++)
  {
    const float *d = &out_data[i * 6];
    if (static_cast<int>(d[5]) != 0) continue;

    float conf = d[4];
    if (conf < CONFIDENCE_THRESHOLDS.at(0)) continue;

    int bx1 = std::clamp(static_cast<int>(d[0] / model_w * orig_w), 0, orig_w - 1);
    int by1 = std::clamp(static_cast<int>(d[1] / model_h * orig_h), 0, orig_h - 1);
    int bx2 = std::clamp(static_cast<int>(d[2] / model_w * orig_w), 0, orig_w - 1);
    int by2 = std::clamp(static_cast<int>(d[3] / model_h * orig_h), 0, orig_h - 1);

    if (bx2 <= bx1 || by2 <= by1) continue;

    vision_tune::utils::detection_result det;
    det.detected = true;
    det.class_id = 0;
    det.score    = conf;
    det.bbox     = cv::Rect(bx1, by1, bx2 - bx1, by2 - by1);
    det.center_x = (bx1 + bx2) / 2.0f;
    det.center_y = (by1 + by2) / 2.0f;
    det.area     = static_cast<float>((bx2 - bx1) * (by2 - by1));
    results.push_back(det);
  }

  return results;
}

static vision_tune::utils::hsv_range get_selected_hsv(const vision_tune::utils::hsv_config &cfg, vision_target target)
{
  switch (target)
  {
  case vision_target::red:
    return cfg.red;
  case vision_target::blue:
    return cfg.blue;
  case vision_target::line:
    return cfg.line;
  }
  return cfg.red;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ProcessNode>());
  rclcpp::shutdown();
  return 0;
}
