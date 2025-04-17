## 目录

[TOC]

# rclcpp/rclcpp.hpp通信库 rcl 
    这个库提供一套创建/管理ROS2节点的API
    rclcpp::init(argc, argv) 初始化 ROS 2 环境
    rclcpp::spin(node_a, node_b)
    rclcpp::spin 是 ROS2 节点的主要执行循环。它负责处理所有与节点相关的回调（例如，消息订阅回调、服务请求回调、定时器回调等）。没有这个，节点将不会执行任何操作。
    rclcpp::shutdown() 清理资源并关闭 ROS 2 环境
    rclcpp::ok() 判断ROS2环境是否异常，如shutdown或者收到终止信号
    rclcpp::TimerBase::SharedPtr 定时器智能指针
    rclcpp::Publisher<std_msgs::msg::string>::SharedPtr 发布者智能指针
**注意**：spin的参数node必须是节点的智能指针std::make_shared<XXX>()
这里XXX写的是Node类或其子类，
如rclcpp::spin(std::make_shared<ServiceClientNode>());



## rclcpp::Node节点成员函数

- create_publisher<std_msgs::msg::String> (话题名, 栈高度)创建话题
- create_wall_timer(500ms, 回调函数)
```
subscriber = this->create_subscription("kami_topic_test", 100, this->subscribeTopic);     
// 上面的代码是错误的，因为
1、ros2创建订阅/发布者都是需要指定话题类型的，这里缺少模板参数类型
2、rclcpp::create_subscription 的最后一个参数必须是对象而不是函数指针
    ，因为单纯的函数不能引用this，而只有用this才能掉用node的方法
3、函数的参数必须为share_ptr类型，这样C++编译才能通过

//所以，正确代码为
m_subscriber = this->create_subscription<std_msgs::msg::String>(
    "kami_topic_test",
    100, 
    [this](std_msgs::msg::String::SharedPtr msg) {
        this->dealTopic(msg);
    });   
```

# std_msgs/msg/string.hpp字符串消息类型
ROS2中的参数通常加了智能指针，如std_msgs::msg::String::SharePtr

std::make_shared 是一个方便且高效的方式来创建 std::shared_ptr

**注意**：在C++中，几乎不存在游离的函数，如果一个函数不是在本类中定义的，基本上都会在前面有一个域名::，如std::make_shared

## 自定义消息类型

包含的时候需要用#include "interface/srv/add_two_ints.hpp"

而我写的数据结构文件为interface/srv/AddtwoInts.srv，ros2编译的时候自动把其编译为小写蛇形命名法的头文件

如何编译一个消息类型包，和创建功能包一样ros2 pkg create --build-type ament_cmake --license Apache-2.0 interface，然后修改目录结构，删除src和include，添加srv action topic目录，修改cmakelist和package.xml文件
在package.xml文件中添加：
```
#这是一个 ROS2 包，提供了用于生成消息、服务和动作接口代码的工具。当你的包定义了自定义的消息、服务或动作时，你需要这个包来生成相应的代码。
<build_depend>rosidl_default_generators</build_depend>

#生成的消息、服务和动作接口所需的运行时库。没有这个包，生成的代码将无法在运行时正确工作。
<exec_depend>rosidl_default_runtime</exec_depend>

#这是一个逻辑分组，用于标识包含 ROS 2 接口定义（消息、服务、动作）的包。将包分组有助于构建系统更好地管理依赖关系和构建顺序
<member_of_group>rosidl_interface_packages</member_of_group>
```
在CMakelists.txt中添加自定义的消息结构文件：
```
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)
```

# C/S通信
client用rclcpp::Client，service用rclcpp::Service，用<消息类型>作为模板参数
rclcpp::Client<nterface::srv::AddTwoInts>::SharedPtr client;
rclcpp::Service<nterface::srv::AddTwoInts>::SharedPtr service;
m_client = this->create_client<interface::srv::AddTwoInts>("add_two_ints");
m_service = this->create_service<interface::srv::AddTwoInts>("add_two_ints");
在调用rclcpp::spin的时候的参数则是std::make_shared(Node类)

**注意**：C/S通信用的async_send_request的参数是智能指针，相对于topic通信用的publish的参数则是对象。

    原因在于client期望在某个时间点接收到响应。这种操作是异步的，可能涉及多个线程或回调，生命周期更长，调用更复杂，服务请求可能需要在多个地方共享（如主线程和异步线程）。
    而发布者就是直接发布数据，生命周期通常是短暂的，发布后即可丢弃。智能指针（如 std::shared_ptr）会引入额外的开销，包括引用计数的管理和内存分配。

# 日志记录
this->get_logger() 时，它返回的就是这个与节点实例关联的日志记录器。
而如果使用rclcpp::get_logger("rclcpp")则表示某个命名模块的日志，通过这个字符串，可以分类日志

判断服务是否开启
```
while (!client->wait_for_service(1s)) {                                                   
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
}
```
**原理**：ROS 2 使用 DDS 作为底层通信中间件。DDS 提供了一种服务发现机制，允许节点动态地发现其他节点及其提供的服务。
wait_for_service在接受数据直接会先发现其他节点及其提供的服务，如果这个时候服务还没启动，根据设置的等待时间会继续等待直到结束。
我们这里是无限循环等待，定时检查rclcpp::ok返回false才退出

# C/S 客户端接收响应
1. 客户端发送数据之后使用spin_until_future_complete阻塞等待响应，但是这个接口和spin一样，需要使用node作为参数，也就是对node本身进行事件循环直到得到result
   
   **注意**：这里的this->get_node_base_interface()就是获取node基础接口，可用于事件循环处理。
    ```
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);             
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }
    ```
2. 使用future_result.wait_for等待响应，时间到了或者收到返回结果，则判断future_result.valid()是否已经获取成功，成功则通过get获取到response的智能指针。但是这个方法并没有启动bin事件循环，测试会卡住在get方法，get是阻塞的。因此需要使用一个线程发送数据，主要线程进行spin事件循环监听。测试不可行    


# 测试服务端
通过命令行访问服务并且指定数据类型和数据进行测试

ros2 service call /add_two_ints interface/srv/AddTwoInts "{a: 2, b: 3}"

# Action动作服务

## 动作服务器的头文件
#include "rclcpp_action/rclcpp_action.hpp"

你会发现动作服务器和另外两个通信用的头文件不同，不使用rclcpp的接口，因为动作是话题和服务上层的抽象。

但是我们依然需要使用rclcpp的node进行事件循环，所以动作服务器需要使用get_node_base_interface获取节点基础接口用于注册事件

## 客户端注册必要的几个函数
ROS 2 的动作服务器（rclcpp_action::create_server）需要与节点的底层功能交互，例如：
- 时间管理：通过 get_node_clock_interface() 获取当前时间，用于超时处理。
- 日志记录：通过 get_node_logging_interface() 记录服务器运行时的信息。
- 异步操作：通过 get_node_waitables_interface() 管理异步任务。
- 基础功能：通过 get_node_base_interface() 获取节点的基本信息。

所以注册的几个参数如下
```
this->client = rclcpp_action::create_client<MoveCircle>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "move_circle");
```

## ROS2的频率循环
rclcpp::Rate loop_rate(1); //这里是每秒执行几次，即Hz
loop_rate.sleep(); //上一次sleep到这一次sleep如果还没到指定的频率，则等待

action动作类型包含三个部分：
- 目标（Goal）：客户端发送给服务器的目标，例如机器人需要移动的参数。
- 结果（Result）：服务器完成目标后返回的结果。
- 反馈（Feedback）：在执行目标过程中，服务器向客户端提供的中间反馈。

![示例GIF](images/actioncs_model.gif)
>如上图所示，动作服务是更高一层的抽象，而底层用的还是话题和服务：

>客户端发送Goal请求后服务器端即刻返回响应，客户端接受到响应之后在发送一个Result请求，服务器端收到请求注册回调函数，启动动作并发起一个feedback话题，直到运动结束，调用Result回调函数返回结果。

## 动作服务器的三个回调函数的关系
1. 客户端发送一个目标请求，动作服务器调用 handle_goal 回调函数来处理该请求。
2. 根据 handle_goal 的返回值：
如果返回 ACCEPT_AND_EXECUTE 或 ACCEPT_AND_DEFER，目标被接受，execute 函数会被调用，execute返回值为void。
如果返回 REJECT，目标被拒绝，execute 函数不会被调用。
3. 动作服务器调用cancel_handle回调函数来处理取消请求
通过return rclcpp_action::CancelResponse::ACCEPT;或者REJECT表示结果

## 动作服务端的回调函数参数
三个回调函数共同使用一个句柄 rclcpp_action::ServerGoalHandle<自定义类型>;
自定义类型有三个成员Feedback Result Goal

有意思的是，三个回调函数里cancel_handle和execute_handle的入参完全一致，而goal_handle的参数不同，如下：
```
cancel_handle(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveCircle>> handle)
execute_handle(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveCircle>> handle)
goal_handle(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const MoveCircle::Goal> request_goal)
/* 
    说明在goal_handle之前还未得到完整的Servergoalhandle句柄，直到目标被EXECUTE才有
    在其他两个回调中，可以通过handle->get_goal()得到目标请求数据
*/
/*
    从上到下分别的返回类型为
    rclcpp_action::CancelResponse // REJECT ACCEPT
    void
    rclcpp_action::GoalResponse // REJECT ACCEPT_AND_EXECUTE ACCEPT_AND_DEFER
*/
```

## 客户端的回调函数参数
可以看到feedback和goal的回调函数都有一个handle句柄，feedback多了一个feedback的参数

而result_callback是最后一个result的回调函数，用的参数是WrappedResult，并不是我们自定义的参数
```
void feedback_callback(rclcpp_action::ClientGoalHandle<MoveCircle>::SharedPtr,
                  std::shared_ptr<const MoveCircle::Feedback> feedback_msg)
void goal_response_callback(rclcpp_action::ClientGoalHandle<MoveCircle>::SharedPtr goal_handle)
void result_callback(rclcpp_action::ClientGoalHandle<MoveCircle>::WrappedResult ret_msg)
```

## 客户端和服务器注册回调函数的区别
客户端通过options绑定回调函数，并在async_send_goal的时候作为参数使用
```
auto send_goal_options = rclcpp_action::Client<MoveCircle>::SendGoalOptions();
using namespace std::placeholders;

send_goal_options.goal_response_callback =
    std::bind(&ActionClientNode::goal_response_callback, this, _1);
send_goal_options.feedback_callback =
    std::bind(&ActionClientNode::feedback_callback, this, _1, _2);
send_goal_options.result_callback =
    std::bind(&ActionClientNode::result_callback, this, _1);

auto send_msg = MoveCircle::Goal();
send_msg.enable = true;
auto future_result = this->client->async_send_goal(send_msg, send_goal_options);
```
服务器是在create的时候直接创建回调函数

也就是说客户端的回调作用于一次请求上，而服务器端回调作用于整个动作服务
```
using namespace std::placeholders;
this->m_rclcpp_action = rclcpp_action::create_server<MoveCircle>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "move_circle",
    std::bind(&ActionServerNode::goal_handle, this, _1, _2),
    std::bind(&ActionServerNode::cancel_handle, this, _1),
    std::bind(&ActionServerNode::execute_handle, this, _1));
```
