#!/usr/bin/python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='pointcloud_compressor',
      namespace='pointcloud_compressor',
      executable='pointcloud_compressor',
      name='pointcloud_compressor',
      output='screen',
      remappings=[
        ('/points', '/os_cloud_node/points'),
        ('/points_compressed', '/os_cloud_node/points_compressed')
      ],
      parameters=[
        {
          'downsample_resolution': 0.0,
          'encoding_speed': 5,
          'decoding_speed': 2,
          'quantization_bits': 11
        }
      ]
    )
  ])
