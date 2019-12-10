# ROS2 YAML example for unknown node name

This repo is a simple package that demonstrates the ability of ROS2 launch routine to load parameters from YAML file with no node name.


In `main.cpp` file we define a simple class that derives from `rclcpp::Node`.
 
The only purpose of this class is to printout the received parameters:

```c++
class MainNode : public rclcpp::Node
{
public:
    MainNode() : rclcpp::Node("node", rclcpp::NodeOptions())
    {
        declare_parameter("param_int", 0);  // Default value of 0
        declare_parameter("param_string", "default_text");   // Default string is given
        
        RCLCPP_INFO(get_logger(), "Integer parameter: %d", get_parameter("param_int").as_int());
        RCLCPP_INFO(get_logger(), "String  parameter: %s", 
                                             get_parameter("param_string").as_string().c_str());
    }
};
```


We define a YAML file with parameters:
```YAML
/**: # This will put all the parameters in every node
  ros__parameters:
    param_int: 5
    param_string: "Different text from the YAML file"
```

Instead of the node name we put **double asterisk**.

Next, we define the launch file with 3 nodes based on `main.cpp`:
* With no namespace and no node parameters
* With defined namespace and parameters from our file
* With no namespace and parameters from our file
 
```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            node_namespace="",
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='node_without_params',
            parameters=[],
            output='screen'),
        Node(
            node_namespace="some_namespace",
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='node_with_params',
            parameters=[os.path.join(
                get_package_share_directory('ros2-yaml-unnamed-node'),
                'param', 'test.param.yaml')],
            output='screen'),
        Node(
            package='ros2-yaml-unnamed-node',
            node_executable='main',
            node_name='another_node_with_same_params',
            parameters=[os.path.join(
                get_package_share_directory('ros2-yaml-unnamed-node'),
                'param', 'test.param.yaml')],
            output='screen'),

    ])

```


 
 Launching the file yields the following output:
 ```
✔ yossi🤖laptop-yo ~:
└─▶ $ ros2 launch ros2-yaml-unnamed-node main.launch.py 
[INFO] [launch]: All log files can be found below /home/yossi/.ros/log/2019-12-10-20-48-49-309370-laptop-yo-20838
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [main-1]: process started with pid [20859]
[INFO] [main-2]: process started with pid [20860]
[INFO] [main-3]: process started with pid [20861]
[main-2] [INFO] [another_node_with_same_params]: Integer parameter: 5
[main-2] [INFO] [another_node_with_same_params]: String  parameter: Different text from the YAML file
[main-1] [INFO] [some_namespace.node_with_params]: Integer parameter: 5
[main-1] [INFO] [some_namespace.node_with_params]: String  parameter: Different text from the YAML file
[main-3] [INFO] [node_without_params]: Integer parameter: 0
[main-3] [INFO] [node_without_params]: String  parameter: default_text

```

As can be seen, the parameters from the YAML file are loaded to both nodes without us having to explicitly define the node name in the YAML file.
