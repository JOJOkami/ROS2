1. 创建工作空间
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. 创建控制节点功能包（C++），指定开源证书
```
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 kami_robot
```

3. 新建cpp代码文件kami\_robot\_control.cpp

```
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"

class KamiRobotControlNode : public rclcpp::Node
{
    public:
        KamiRobotControlNode()
        : Node("kami_robot_control")                             // ROS2节点父类初始化
        {
            while(rclcpp::ok())                                  // ROS2系统是否正常运行
            {
                RCLCPP_INFO(this->get_logger(), "Hello World");  // ROS2日志输出
                sleep(1);                                        // 休眠控制循环时间
            }
        }
};

int main(int argc, char * argv[])                               
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);        
    // 创建ROS2节点对象并进行初始化                 
    rclcpp::spin(std::make_shared<KamiRobotControlNode>()); 
    // 关闭ROS2 C++接口
    rclcpp::shutdown();                               
    return 0;
}
```

4. 修改cmakelist文件

    主要添加以下内容：

    依赖包查找和指定依赖

    定义可执行程序名并且install到对应位置，lib下的\${PROJECT\_NAME}就是功能包名，也就是一个目录
```
# 查找依赖
find_package(rclcpp REQUIRED)
# 定义可执行程序
add_executable(kami_robot_control src/kami_robot_control.cpp)
# 指定可执行程序的依赖
ament_target_dependencies(kami_robot_control rclcpp)
# 将可执行程序放到对应位置，TARGETS后面跟目标程序，DESTINATION后面跟目标目录
install(TARGETS
  kami_robot_control
  DESTINATION lib/${PROJECT_NAME})
```

5. 对工作空间内的节点进行编译

    cd \~/ros2\_ws/

    colcon build

    如果只要编译某个功能包如learning\_service执行，后面可跟多个功能包目录
    
    colcon build \--paths src/learning\_service

6. 设置环境变量

    source install/local\_setup.sh    \#如果是最终的所有功能包加载，应使用setup.sh

    如果你在 package\_a 的开发目录中运行 source
    install/local\_setup.sh，环境变量中只会包含 package\_a 的路径，而不会包含 package\_b 的路径。

    （1）单包开发场景

    假设你的工作空间中有两个功能包：package\_a 和 package\_b。

    你在 package\_a 的开发目录中运行以下命令：

    bash

    cd workspace/src/package\_a

    colcon build \--packages-select package\_a

    source ../install/local\_setup.sh

    此时，local\_setup.sh 会只设置 package\_a 的环境变量，而不会影响
    package\_b。

    （2）切换功能包

    如果你切换到 package\_b 的开发目录，并重新加载 local\_setup.sh：

    bash

    cd workspace/src/package\_b

    colcon build \--packages-select package\_b

    source ../install/local\_setup.sh

    此时，环境变量会更新为 package\_b 的路径，而不会包含 package\_a 的路径。


7. 执行节点
   
   ros2 run kami\_robot kami\_robot\_control
