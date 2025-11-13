from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler,IncludeLaunchDescription
from launch.event_handlers import OnProcessStart,OnProcessExit
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import Command,PathJoinSubstitution,LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description(): 
    
    pkg_arm ='arm_description'
    urdf_file='arm_model.urdf.xacro'
    pkg_description=FindPackageShare(pkg_arm) 

    # Initialization of directories
    xacro_file_GZ =PathJoinSubstitution([pkg_description,'urdf','robots',urdf_file]) 
          
    robot_description_content=ParameterValue(Command([
          'xacro',' ',xacro_file_GZ]),
          value_type=str)
     
    robot_controllers=PathJoinSubstitution([FindPackageShare('arm_gazebo'),'config','simple_controller.yaml'])
    
    #Declaration Gazebo and world
    world_file_name = 'empty_world.sdf'
    world = os.path.join(get_package_share_directory('arm_gazebo'), 'world', world_file_name)
     
    declare_gz_args_cmd = DeclareLaunchArgument(
          name = 'gz_args',
          default_value = ['-r -v 4 ',world],
          description = 'Defining the world for robot model'
     )
    
    gz_args = LaunchConfiguration('gz_args')

     #Gazebo node
    gazebo_launch = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'])
               ]),
                launch_arguments={'gz_args': gz_args}.items()
          )

     # Robot State Publisher to generate the /robot state topic with URDF data
    robot_state_publisher=Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name='robot_state_publisher_arm',
		output='screen',
		parameters=[{
			'robot_description':robot_description_content}]
     )

     #Spawn the robot
    spawn_entity_robot=Node(
        package='ros_gz_sim',
        executable='create',
        arguments   = ['-topic', 'robot_description' , '-name', 'arm_6dof'],
        output='screen'
     )

    joint_state_broadcaster_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=['joint_state_broadcaster']
     )

    controller_spawner=Node(
          package="controller_manager",
          executable="spawner",
          arguments=["arm_controller",
          '--param-file',robot_controllers
          ]
     )   

    delay_joint_state_broadcaster_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
     
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[controller_spawner],
        )
    )

     
    return LaunchDescription([   
          declare_gz_args_cmd,
          delay_joint_state_broadcaster_after_joint_state_broadcaster_spawner,
          delay_joint_state_broadcaster_after_robot_controller_spawner,
          gazebo_launch,
          robot_state_publisher,
          spawn_entity_robot
    ])

''' 
     
     
     #Create the gazebo world
     gazebo_node = ExecuteProcess(
          cmd=['gz' , 'sim', '--verbose', world],
               output='screen')
     
     # Launch joint state broadcaster
     load_joint_state_broadcaster_cmd = ExecuteProcess(
          cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
               'joint_state_broadcaster'],
          output='screen')
     
     # Start arm controller
     load_arm_controller_cmd = ExecuteProcess(
          cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
               'arm_controller'],
          output='screen')
     
      
     control_node=Node(
          package='controller_manager',
          executable='ros2_control_node',
          parameters=[{'robot_description': robot_description_content},
                      robot_controllers],
          output='both'
     )
       
     
     
    delayed_robot = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[control_node],
        )
    )

     delay_rviz_after_joint_state_broadcaster_spawner=RegisterEventHandler(
          event_handler=OnProcessExit(
               target_action=joint_state_broadcaster_spawner,
               on_exit=[rviz_arm_node]
          )
     )
     
    
     delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
     
     trajectory_node = Node(
          package="ros2_control_demos_example_7",
          executable="send_trajectory",
          parameters =[{'robot_description': robot_description_content}]     
          )

          spawn_entity_robot,
        joint_state_broadcaster_spawner,
        controller_spawner

        
          '''
