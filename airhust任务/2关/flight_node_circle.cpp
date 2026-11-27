// flight_node_circle.cpp
// AirHUST 26 第 2 关：绕圈任务完整实现（基于官方 template.h，离散航点方案，即文档"思路 A"）
//
// 任务流程：起飞 -> 到达圆周起点 -> 离散航点绕行一圈 -> 悬停 -> 降落
//
// 使用方式（二选一）：
//   方式 1：在 CMakeLists.txt 中新增可执行目标：
//     add_executable(flight_node_circle src/flight_node_circle.cpp)
//     target_link_libraries(flight_node_circle ${catkin_LIBRARIES})
//     并把本文件放到包的 src/ 目录下。
//   方式 2：直接把本文件内容替换进原来的 src/template.cpp，不改 CMakeLists.txt。
//
// 与模板的关系：
//   - 不修改 template.h。所有绕圈状态（航点编号、圈数、超时计时）都在本文件内定义。
//   - 完全沿用模板的分工约定：任务函数只填全局 setpoint_raw，发布由主循环 switch
//     之后的那一次 publish 统一完成，本文件不额外 publish。
//   - 绕圈参数全部通过 nh.param 从参数服务器读取（由 circle_demo.launch 加载
//     circle_config.yaml 注入），满足"至少一个绕圈参数放入 YAML"的要求。
//
// 关键正确性说明：
//   1. mission_pos_cruise(x, y, z, target_yaw, error_max) 会把 XYZ 加上起飞点位置
//      (init_position_*_take_off)，因此本文件里所有目标坐标都是"相对起飞点的偏移"。
//      圆周圆心取 (0, 0)，即圆心在起飞点正上方，公式 x = r*cos(theta), y = r*sin(theta),
//      z = ALTITUDE 可直接喂给该函数，无需再换算参考系。
//   2. 该函数的 yaw 判定阈值是硬编码 0.1 rad，且是"世界系绝对航向"。本实现全程固定
//      机头 yaw = 0，避开 ±pi 归一化问题。若以后要让机头沿切线方向跟随，必须自行把
//      yaw 误差归一化到 [-pi, pi] 再比较。
//   3. 航点编号 k 取 1..N。k = N 时 theta = direction * 2*pi，其 cos/sin 与起点相同，
//      因此"到达第 N 个航点"天然等价于"回到圆周起点"，一圈自动闭合，不需要额外特判。
//   4. 每个 case 都带超时兜底。若 launch 未注入 err_max，模板默认值为 0，位置任务
//      将永远判不到到达，此时由超时兜底保证流程仍可向前（现场演示前务必注入 err_max）。

#include <template.h>

#include <clocale>
#include <cstdlib>
#include <iostream>
#include <cmath>

using std::cout;
using std::endl;

// ---------------- 绕圈参数（默认值 + 由参数服务器覆盖，单位见注释） ----------------
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;

float circle_radius = 1.5f;        // 单位: 米(m)。圆周半径，圆心在起飞点正上方
int   circle_waypoints = 12;       // 单位: 个。一圈离散化成的航点数 N，越小越像多边形
int   circle_direction = 1;        // 1 = 逆时针(theta 增大方向)，-1 = 顺时针
float circle_err = 0.2f;           // 单位: 米(m)。单个航点的到达容差(三轴同阈值)
float circle_start_yaw = 0.0f;     // 单位: 弧度(rad)。全程固定的机头朝向
float circle_hover_time = 3.0f;    // 单位: 秒(s)。绕圈结束后悬停时长
float circle_lap_timeout = 120.0f; // 单位: 秒(s)。绕圈阶段整体超时兜底
float circle_start_timeout = 30.0f;// 单位: 秒(s)。飞向圆周起点阶段的超时兜底

// ---------------- 绕圈运行状态（只在本文件使用，不进 template.h） ----------------
int waypoint_index = 1;            // 当前正在飞行的航点编号 k (1..N)
int lap_completed = 0;             // 已完成圈数，作为绕圈进度的日志证据
ros::Time lap_start_time;          // 进入绕圈阶段的时刻，用于整体超时

void print_param()
{
  std::cout << "=== 控制参数 ===" << std::endl;
  std::cout << "err_max: " << err_max << std::endl;
  std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
  std::cout << "if_debug: " << if_debug << std::endl;
  std::cout << "--- 绕圈参数 ---" << std::endl;
  std::cout << "circle_radius (m): " << circle_radius << std::endl;
  std::cout << "circle_waypoints (个): " << circle_waypoints << std::endl;
  std::cout << "circle_direction (1逆时针/-1顺时针): " << circle_direction << std::endl;
  std::cout << "circle_err (m): " << circle_err << std::endl;
  std::cout << "circle_start_yaw (rad): " << circle_start_yaw << std::endl;
  std::cout << "circle_hover_time (s): " << circle_hover_time << std::endl;
  std::cout << "circle_lap_timeout (s): " << circle_lap_timeout << std::endl;
  if (if_debug == 1) cout << "自动offboard" << std::endl;
  else cout << "遥控器offboard" << std::endl;
}

// 由航点编号 k 计算圆周目标点（相对起飞点的偏移坐标，直接喂给 mission_pos_cruise）
// theta_k = theta_0 + s * 2*pi*k / N，theta_0 取 0
void circle_waypoint(int k, float &wx, float &wy)
{
  double theta = (double)circle_direction * 2.0 * M_PI * (double)k / (double)circle_waypoints;
  wx = circle_radius * (float)cos(theta);
  wy = circle_radius * (float)sin(theta);
}

int main(int argc, char **argv)
{
  // 防止中文输出乱码
  setlocale(LC_ALL, "");

  ros::init(argc, argv, "template");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

  ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // 20Hz 发布，满足 500ms 心跳要求
  ros::Rate rate(20);

  // 参数读取：模板原有参数 + 绕圈参数（launch/yaml 注入，见 circle_config.yaml）
  nh.param<float>("err_max", err_max, 0.2f);
  nh.param<float>("if_debug", if_debug, 0);
  nh.param<float>("circle_radius", circle_radius, 1.5f);
  nh.param<int>("circle_waypoints", circle_waypoints, 12);
  nh.param<int>("circle_direction", circle_direction, 1);
  nh.param<float>("circle_err", circle_err, 0.2f);
  nh.param<float>("circle_start_yaw", circle_start_yaw, 0.0f);
  nh.param<float>("circle_hover_time", circle_hover_time, 3.0f);
  nh.param<float>("circle_lap_timeout", circle_lap_timeout, 120.0f);
  nh.param<float>("circle_start_timeout", circle_start_timeout, 30.0f);
  print_param();

  if (circle_waypoints < 4)
  {
    std::cout << "circle_waypoints 太小(<4)，圆周会退化成几个点，已改为 12" << std::endl;
    circle_waypoints = 12;
  }
  if (circle_direction != 1 && circle_direction != -1) circle_direction = 1;

  int choice = 0;
  std::cout << "1 to go on , else to quit" << std::endl;
  std::cin >> choice;
  if (choice != 1) return 0;
  ros::spinOnce();
  rate.sleep();

  // 等待连接飞控
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  // ---------------- 预备阶段：切 OFFBOARD、解锁、起飞爬升（与模板一致，勿改） ----------------
  while (ros::ok())
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
    {
      if (if_debug == 1)
      {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
          ROS_INFO("Offboard enabled");
        }
      }
      else
      {
        ROS_INFO("Waiting for OFFBOARD mode");
      }
      last_request = ros::Time::now();
    }
    else
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    // 到达巡航高度附近后悬停 1 秒再进入任务主循环（与模板一致）
    if (fabs(local_pos.pose.pose.position.z - (ALTITUDE + init_position_z_take_off)) < 0.2)
    {
      if (ros::Time::now() - last_request > ros::Duration(1.0))
      {
        mission_num = 1;
        last_request = ros::Time::now();
        break;
      }
    }

    mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();
  }

  // ---------------- 任务主循环 ----------------
  while (ros::ok())
  {
    ROS_WARN("mission_num = %d", mission_num);

    switch (mission_num)
    {
      // mission 1: 起飞确认（正常判据 + 3 秒超时兜底，沿用模板结构）
      case 1:
        if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
        {
          if (lib_time_record_func(1.0, ros::Time::now()))
          {
            mission_num = 2;
            last_request = ros::Time::now();
          }
        }
        else if (ros::Time::now() - last_request >= ros::Duration(3.0))
        {
          mission_num = 2;
          last_request = ros::Time::now();
          lib_time_init_flag = true;
        }
        break;

      // mission 2: 飞往圆周起点。theta_0 = 0，起点 = (r, 0, ALTITUDE)
      case 2:
        if (mission_pos_cruise(circle_radius * cos(0.0), circle_radius * sin(0.0),
                               ALTITUDE, circle_start_yaw, circle_err))
        {
          ROS_INFO("已到达圆周起点，开始绕行 %d 个航点", circle_waypoints);
          waypoint_index = 1;
          lap_start_time = ros::Time::now();
          mission_num = 3;
          last_request = ros::Time::now();
        }
        else if (ros::Time::now() - last_request >= ros::Duration(circle_start_timeout))
        {
          // 兜底：长时间未到达起点(如 err_max 被配成 0)也进入绕圈，避免整条流程卡死
          ROS_WARN("飞向圆周起点超时，兜底直接进入绕圈阶段");
          waypoint_index = 1;
          lap_start_time = ros::Time::now();
          mission_num = 3;
          last_request = ros::Time::now();
        }
        break;

      // mission 3: 离散航点绕行。k = 1..N，k = N 的目标与起点重合，到达即完成一圈
      case 3:
      {
        float wx = 0, wy = 0;
        circle_waypoint(waypoint_index, wx, wy);
        if (mission_pos_cruise(wx, wy, ALTITUDE, circle_start_yaw, circle_err))
        {
          ROS_INFO("航点 %d/%d 完成 (theta = %.1f 度)",
                   waypoint_index, circle_waypoints,
                   (double)circle_direction * 360.0 * waypoint_index / circle_waypoints);
          waypoint_index++;
          if (waypoint_index > circle_waypoints)
          {
            lap_completed++;
            ROS_INFO("第 %d 圈完成，回到圆周起点", lap_completed);
            mission_num = 4;
            last_request = ros::Time::now();
          }
        }
        if (ros::Time::now() - lap_start_time > ros::Duration(circle_lap_timeout))
        {
          ROS_WARN("绕圈阶段整体超时(%.0f s)，带当前进度进入悬停", circle_lap_timeout);
          mission_num = 4;
          last_request = ros::Time::now();
        }
        break;
      }

      // mission 4: 悬停（直接复用 template.h 的 hover）
      case 4:
        if (hover(circle_hover_time))
        {
          mission_num = 5;
          last_request = ros::Time::now();
        }
        break;

      // mission 5: 降落（直接复用 template.h 的 precision_land）
      case 5:
        if (precision_land())
        {
          mission_num = -1; // 任务结束
          last_request = ros::Time::now();
        }
        break;
    }

    // 全程序唯一的发布点：任务函数只填 setpoint_raw，这里统一发出，保证心跳
    mavros_setpoint_pos_pub.publish(setpoint_raw);
    ros::spinOnce();
    rate.sleep();

    if (mission_num == -1)
    {
      ROS_INFO("任务结束，共完成 %d 圈", lap_completed);
      exit(0);
    }
  }
  return 0;
}
