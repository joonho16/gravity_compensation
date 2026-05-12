#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

// Pinocchio
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

// 패키지 경로 찾기용
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

// ==========================================
// [사용자 하드웨어 설정]
// ==========================================
#define DEVICENAME "/dev/ttyUSB0"
#define BAUDRATE 4500000
#define URDF_FILENAME "my_robot.urdf"

// 모터 ID (1~6번이라고 가정)
const std::vector<uint8_t> MY_DXL_ID = {10, 11, 12, 13, 14, 15};
const uint8_t GRIPPER_DXL_ID = 16; // 그리퍼 모터 ID

// 방향 보정 (모터 회전 방향과 URDF 회전 방향이 반대면 -1)
const std::vector<int> AXIS_DIR = {1, 1, -1, 1, -1, 1};
const std::vector<double> JOINT_OFFSET = {0.0, -1.31, 2.88, 0.0, 0.0, 0.0};

const double TORQUE_CONSTANT = 1.8;  // XM430-W350 기준 (Nm/A)
const double CURRENT_UNIT_MA = 2.69; // 1 unit = 2.69mA

// [중요] URDF 파일 내부의 Joint 이름과 순서 매칭
const std::vector<std::string> JOINT_NAMES = {
    "joint1", // ID 1
    "joint2", // ID 2
    "joint3", // ID 3
    "joint4", // ID 4
    "joint5", // ID 5
    "joint6"  // ID 6
};

// 다이나믹셀 주소 (XM 시리즈 Protocol 2.0)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_CURRENT 102
#define LEN_GOAL_CURRENT 2
#define ADDR_READ_START 128
#define LEN_READ_TOTAL 8
#define ADDR_PRESENT_POS 132
#define LEN_PRESENT_POS 4

class GravityNode : public rclcpp::Node
{
public:
  GravityNode() : Node("gravity_node")
  {
    // 1. URDF 모델 로드
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("gravity_compensation");
    std::string full_urdf_path = package_share_directory + "/urdf/" + std::string(URDF_FILENAME);

    try
    {
      pinocchio::urdf::buildModel(full_urdf_path, model_);
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "URDF 로드 실패! 경로 확인: %s", full_urdf_path.c_str());
      throw;
    }

    RCLCPP_INFO(this->get_logger(), "=== Pinocchio Joint Order ===");
    for (int i = 0; i < model_.njoints; i++)
    {
      // 0번은 보통 universe(고정)이므로 1번부터가 실제 조인트일 수 있음
      // model_.names[i] 와 내 JOINT_NAMES 매칭 확인
      std::cout << "Index " << i << ": " << model_.names[i] << std::endl;
    }

    // [1] 파라미터 선언 (외부에서 수정 가능하게 만듦)
    for (int i = 0; i < 6; i++)
    {
      std::string id_str = std::to_string(i + 1);

      auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
      descriptor.description = "Gravity Gain for Joint " + id_str;
      descriptor.floating_point_range.resize(1);
      descriptor.floating_point_range[0].from_value = 0.0;
      descriptor.floating_point_range[0].to_value = 5.0;
      descriptor.floating_point_range[0].step = 0.01;

      this->declare_parameter("g_gain_" + id_str, gravity_gains_[i], descriptor);
      this->declare_parameter("k_coulomb_" + id_str, friction_k_c_[i]);
      this->declare_parameter("k_viscous_" + id_str, friction_k_v_[i]);
    }

    // [2] 콜백 함수 등록
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&GravityNode::parametersCallback, this, std::placeholders::_1));

    model_.gravity.linear() = Eigen::Vector3d(0.0, 0.0, -9.81);

    data_ = std::make_unique<pinocchio::Data>(model_);
    q_ = Eigen::VectorXd::Zero(model_.nq);

    // 2. 다이나믹셀 초기화
    init_dynamixel();

    // 3. Publisher 생성
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("ec_robot_5/joint_states", 10);

    // 4. 제어 루프 타이머 (1000Hz = 1ms)
    timer_ = this->create_wall_timer(
        1ms, std::bind(&GravityNode::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Gravity Compensation Node Started! (1kHz)");
  }

  ~GravityNode()
  {
    // RCLCPP_WARN(this->get_logger(), "🛑 Simple Damping Stop...");

    // double damping_gain = 0.3;
    // int steps = 200;

    // for (int k = 0; k < steps; k++) {
    //     int comm_result = groupSyncRead_->txRxPacket();
    //     if (comm_result != COMM_SUCCESS) continue;

    //     groupSyncWrite_->clearParam();

    //     for (size_t i = 0; i < MY_DXL_ID.size(); i++) {
    //         uint8_t id = MY_DXL_ID[i];

    //         if (groupSyncRead_->isAvailable(id, 128, 4)) {
    //             int32_t vel_raw = groupSyncRead_->getData(id, 128, 4);
    //             double vel_rpm = vel_raw * 0.229;
    //             double velocity = (vel_rpm * 2.0 * M_PI) / 60.0;

    // velocity *= AXIS_DIR[i];
    // double brake_torque = -damping_gain * velocity;

    //             if (std::abs(velocity) > 0.1) {
    //                 brake_torque = -damping_gain * velocity;
    //             }

    //             if (brake_torque > 0.5) brake_torque = 0.5;
    //             if (brake_torque < -0.5) brake_torque = -0.5;

    // double current_ma = (brake_torque / TORQUE_CONSTANT) * 1000.0;
    // int16_t goal_current = (int16_t)(current_ma / CURRENT_UNIT_MA);
    // goal_current *= AXIS_DIR[i];

    //             uint8_t param[2];
    //             param[0] = DXL_LOBYTE(goal_current);
    //             param[1] = DXL_HIBYTE(goal_current);
    //             groupSyncWrite_->addParam(id, param);
    //         }
    //     }
    //     groupSyncWrite_->txPacket();

    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }

    for (auto id : MY_DXL_ID) {
        packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0);
    }
    portHandler_->closePort();
    RCLCPP_INFO(this->get_logger(), "Finished.");
  }

private:
  void init_dynamixel()
  {
    portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler_->openPort() || !portHandler_->setBaudRate(BAUDRATE))
    {
      RCLCPP_FATAL(this->get_logger(), "포트 열기 실패! 권한이나 경로를 확인하세요.");
      rclcpp::shutdown();
    }

    groupSyncRead_ = std::make_unique<dynamixel::GroupSyncRead>(
        portHandler_, packetHandler_, ADDR_READ_START, LEN_READ_TOTAL);
    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(
        portHandler_, packetHandler_, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);

    for (auto id : MY_DXL_ID)
    {
      packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 0);
      packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, 0); // Current Control
      packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1);
      groupSyncRead_->addParam(id);
    }
    groupSyncRead_->addParam(GRIPPER_DXL_ID);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &param : parameters)
    {
      // 파라미터 이름 파싱 (예: "g_gain_2")
      std::string name = param.get_name();

      // 몇 번 조인트인지 숫자 추출
      // 단순하게 맨 뒤 글자가 숫자라고 가정 (1~6)
      char last_char = name.back();
      if (!isdigit(last_char))
        continue;
      int idx = (last_char - '0') - 1; // 0-based index
      if (idx < 0 || idx >= 6)
        continue;

      if (name.find("g_gain_") != std::string::npos)
      {
        gravity_gains_[idx] = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Joint %d Gravity Gain -> %.2f", idx + 1, gravity_gains_[idx]);
      }
      else if (name.find("k_coulomb_") != std::string::npos)
      {
        friction_k_c_[idx] = param.as_double();
      }
      else if (name.find("k_viscous_") != std::string::npos)
      {
        friction_k_v_[idx] = param.as_double();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  void control_loop()
  {
    portHandler_->clearPort();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);

    auto start = std::chrono::steady_clock::now();
    // [A] 하드웨어 읽기
    int comm_result = groupSyncRead_->txRxPacket();
    auto t1 = std::chrono::steady_clock::now(); // 읽기 끝

    if (comm_result != COMM_SUCCESS)
      return;

    for (size_t i = 0; i < MY_DXL_ID.size(); i++)
    {
      uint8_t id = MY_DXL_ID[i];
      if (groupSyncRead_->isAvailable(id, ADDR_READ_START, LEN_READ_TOTAL))
      {
        // 1. 속도 읽기 (앞 4바이트: 128~131)
        int32_t vel_raw = groupSyncRead_->getData(id, 128, 4);
        double vel_rpm = vel_raw * 0.229; // XM 시리즈 RPM 단위
        double vel_rad = vel_rpm * (2.0 * M_PI / 60.0);
        // v[i] = vel_rad * AXIS_DIR[i];
        v[i] = vel_rad;

        // 2. 위치 읽기 (뒤 4바이트: 132~135)
        int32_t pos_raw = groupSyncRead_->getData(id, 132, 4);
        double pos_rad = (pos_raw - 2048.0) * (2.0 * M_PI / 4096.0);
        // q_[i] = (pos_rad * AXIS_DIR[i]) + JOINT_OFFSET[i];
        q_[i] = pos_rad;
      }
    }
    if (groupSyncRead_->isAvailable(GRIPPER_DXL_ID, ADDR_READ_START, LEN_READ_TOTAL))
    {
      int32_t pos_raw = groupSyncRead_->getData(GRIPPER_DXL_ID, 132, 4);
      double pos_rad = (pos_raw - 2048.0) * (2.0 * M_PI / 4096.0);
      // pos_rad -> 0.0~0.1 사이로 매핑
      if (pos_rad < 0.0)
        pos_rad = 0.0;
      else if (pos_rad > 0.5236)
        pos_rad = 0.5236;
      pos_rad = (pos_rad) * (0.1 - 0.0) / (0.5236 - 0.0) + 0.0;
      gripper_position_ = pos_rad;
    }

    // [B-1] 중력 토크 계산
    Eigen::VectorXd tau_gravity = pinocchio::computeGeneralizedGravity(model_, *data_, q_);

    // [B-2] 마찰 보상 토크 계산
    Eigen::VectorXd tau_friction = Eigen::VectorXd::Zero(model_.nq);
    for (size_t i = 0; i < MY_DXL_ID.size(); i++)
    {
      double velocity = v[i];

      // 쿨롱 마찰 (방향에 따라 상수를 더함)
      // 노이즈로 인한 떨림 방지를 위해 아주 느릴 땐(0.01 rad/s 이하) 적용 안 함
      double friction_c = 0.0;
      if (velocity > 0.01)
        friction_c = friction_k_c_[i];
      else if (velocity < -0.01)
        friction_c = -friction_k_c_[i];

      // 점성 마찰
      double friction_v = friction_k_v_[i] * velocity;

      tau_friction[i] = friction_c + friction_v;
    }

    // [최종 토크] 중력 + 마찰
    // 중요: 마찰 보상은 "내가 움직이려는 힘"을 도와주는 게 아니라,
    // 외력에 의해 움직일 때 저항을 상쇄해야 하므로 부호가 중요합니다.
    // 보통: 토크 명령 = 중력 + sign(속도)*마찰

    for (size_t i = 0; i < MY_DXL_ID.size(); i++)
    {
      tau_gravity[i] = tau_gravity[i] * gravity_gains_[i];
    }

    Eigen::VectorXd tau = tau_gravity + tau_friction;

    auto t2 = std::chrono::steady_clock::now(); // 계산 끝

    for (size_t i = 0; i < MY_DXL_ID.size(); i++)
    {
      double q_curr = q_[i];
      double q_min = model_.lowerPositionLimit[i];
      double q_max = model_.upperPositionLimit[i];

      // 한계점보다 5도(약 0.087rad) 안쪽을 '위험 구간'으로 설정
      double margin = 0.087;

      double wall_torque = 0.0;
      double K_wall = 5.0; // 벽 스프링 상수 (강하게 밀어내는 힘)

      // 상한선에 다다르면 -> 아래로 미는 힘 추가
      if (q_curr > (q_max - margin))
      {
        wall_torque = -K_wall * (q_curr - (q_max - margin));
      }
      // 하한선에 다다르면 -> 위로 미는 힘 추가
      else if (q_curr < (q_min + margin))
      {
        wall_torque = K_wall * ((q_min + margin) - q_curr);
      }

      // 최종 토크에 가상의 벽 힘 추가
      tau[i] += wall_torque;
    }

    // [C] 하드웨어 쓰기
    groupSyncWrite_->clearParam();

    if (last_goal_currents_.size() != MY_DXL_ID.size())
    {
      last_goal_currents_.resize(MY_DXL_ID.size(), 0);
    }

    for (size_t i = 0; i < MY_DXL_ID.size(); i++)
    {
      double torque = tau[i];

      double max_effort = model_.effortLimit[i];

      // 토크가 한계를 넘으면 잘라버림 (Safety Clamp)
      if (torque > max_effort)
        torque = max_effort;
      if (torque < -max_effort)
        torque = -max_effort;

      double current_ma = (torque / TORQUE_CONSTANT) * 1000.0;
      int16_t goal_current = (int16_t)(current_ma / CURRENT_UNIT_MA);

      // goal_current *= AXIS_DIR[i];

      last_goal_currents_[i] = goal_current;

      uint8_t param[2];
      param[0] = DXL_LOBYTE(goal_current);
      param[1] = DXL_HIBYTE(goal_current);
      groupSyncWrite_->addParam(MY_DXL_ID[i], param);
    }
    groupSyncWrite_->txPacket();

    auto end = std::chrono::steady_clock::now(); // 루프 끝

    auto d_read = std::chrono::duration_cast<std::chrono::microseconds>(t1 - start).count();
    auto d_calc = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    auto d_total = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // 100번에 한 번만 출력 (터미널 도배 방지)
    static int p_count = 0;
    if (p_count++ % 100 == 0)
    {
      std::cout << "Read: " << d_read << "us | Calc: " << d_calc << "us | Total: " << d_total << "us" << std::endl;
    }

    // [D] 시각화 데이터 발행 (100Hz)
    static int count = 0;
    if (count++ % 10 == 0)
    {
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = this->now();
      for (size_t i = 0; i < MY_DXL_ID.size(); i++)
      {
        msg.name.push_back(JOINT_NAMES[i]); // 매칭된 이름 사용
        q_[i] = (q_[i] * AXIS_DIR[i]) - JOINT_OFFSET[i];
        msg.position.push_back(q_[i]);
        msg.effort.push_back(tau[i]);
      }

      msg.name.push_back("gripper");
      msg.position.push_back(-1 * gripper_position_);
      msg.effort.push_back(0.0);

      joint_pub_->publish(msg);
    }
  }

  // Kc: 정지 마찰 (단위: Nm), Kv: 점성 마찰 (단위: Nm / (rad/s))
  std::vector<double> friction_k_c_ = {0.0, 0.02, 0.02, 0.01, 0.01, 0.0};
  std::vector<double> friction_k_v_ = {0.001, 0.01, 0.01, 0.0, 0.0, 0.0};

  std::vector<double> gravity_gains_ = {1.0, 0.75, 0.75, 0.5, 0.3, 0.3};

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  std::unique_ptr<dynamixel::GroupSyncRead> groupSyncRead_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_;
  double gripper_position_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<int16_t> last_goal_currents_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityNode>());
  rclcpp::shutdown();
  return 0;
}