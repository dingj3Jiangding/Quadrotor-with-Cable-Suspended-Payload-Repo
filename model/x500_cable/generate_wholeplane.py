# Python script to generate a flexible cable model with adjustable parameters

import os

# Parameters
num_segments = 30
segment_length = 0.016
cable_top_z = 0.2
sphere_radius = 0.003
cylinder_radius = 0.003
sphere_mass = 0.005
cylinder_mass = 0.01
end_sphere_radius = 0.015
end_sphere_mass = 0.08
cylinder_length = 0.01

# Cable base definition
sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_cable'>
    <include merge='true'>
      <uri>x500</uri>
    </include>
    <link name="cable_base_link">
      <pose>0 0 {cable_top_z} 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="base_link_visual">
        <geometry><box><size>0.04 0.04 0.02</size></box></geometry>
        <material><ambient>1 0 0 1</ambient></material>
      </visual>
      <collision name="base_link_collision">
        <geometry><box><size>0.04 0.04 0.02</size></box></geometry>
      </collision>
      <self_collide>true</self_collide>
    </link>
    <joint name="joint_x500_to_cable" type="fixed">
      <parent>base_link</parent>
      <child>cable_base_link</child>
      <pose>0 0 {cable_top_z} 0 0 0</pose>
    </joint>

    <link name="segment_{1}_sphere">
      <pose>0 0 0.19 0 0 0</pose>
      <inertial><mass>{sphere_mass}</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <visual name="segment_{1}_sphere_visual">
        <geometry><sphere><radius>{sphere_radius}</radius></sphere></geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{1}_sphere_collision">
        <geometry><sphere><radius>{sphere_radius}</radius></sphere></geometry>
      </collision>
      <self_collide>true</self_collide>
    </link>

    <joint name="segment_{1}_joint" type="revolute">
      <parent>{'cable_base_link'}</parent>
      <child>segment_{1}_sphere</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-3.14</lower><upper>3.14</upper></limit>
        <dynamics><damping>20</damping><friction>1</friction></dynamics></axis>
    </joint>

    <link name="segment_{1}_cylinder">
      <pose>0 0 0.182 0 0 0</pose>
      <inertial><mass>{cylinder_mass}</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <visual name="segment_{1}_cylinder_visual">
        <geometry><cylinder><radius>{cylinder_radius}</radius><length>{cylinder_length}</length></cylinder></geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{1}_collision">
        <geometry><cylinder><radius>{cylinder_radius}</radius><length>{cylinder_length}</length></cylinder></geometry>
      </collision>
      <self_collide>true</self_collide>
    </link>

    <joint name="segment_{1}_sphere_to_cylinder" type="revolute">
      <parent>segment_{1}_sphere</parent>
      <child>segment_{1}_cylinder</child>
      <axis><xyz>1 0 0</xyz><limit><lower>-3.14</lower><upper>3.14</upper></limit>
        <dynamics><damping>20</damping><friction>1</friction></dynamics></axis>
    </joint>
"""

# Generate segments
for i in range(2, num_segments + 1):
    z_sphere = cable_top_z - i * segment_length + 0.01
    z_cylinder = z_sphere - 0.008

    sdf += f"""
    <link name="segment_{i}_sphere">
      <pose>0 0 {z_sphere:.3f} 0 0 0</pose>
      <inertial><mass>{sphere_mass}</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <visual name="segment_{i}_sphere_visual">
        <geometry><sphere><radius>{sphere_radius}</radius></sphere></geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{i}_sphere_collision">
        <geometry><sphere><radius>{sphere_radius}</radius></sphere></geometry>
      </collision>
      <self_collide>true</self_collide>
    </link>

    <joint name="segment_{i}_joint" type="revolute">
      <parent>{'cable_base_link' if i == 1 else f'segment_{i-1}_cylinder'}</parent>
      <child>segment_{i}_sphere</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-3.14</lower><upper>3.14</upper></limit>
        <dynamics><damping>20</damping><friction>1</friction></dynamics></axis>
    </joint>

    <link name="segment_{i}_cylinder">
      <pose>0 0 {z_cylinder:.3f} 0 0 0</pose>
      <inertial><mass>{cylinder_mass}</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>
      <visual name="segment_{i}_cylinder_visual">
        <geometry><cylinder><radius>{cylinder_radius}</radius><length>{cylinder_length}</length></cylinder></geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{i}_collision">
        <geometry><cylinder><radius>{cylinder_radius}</radius><length>{cylinder_length}</length></cylinder></geometry>
      </collision>
      <self_collide>true</self_collide>
    </link>

    <joint name="segment_{i}_sphere_to_cylinder" type="revolute">
      <parent>segment_{i}_sphere</parent>
      <child>segment_{i}_cylinder</child>
      <axis><xyz>1 0 0</xyz><limit><lower>-3.14</lower><upper>3.14</upper></limit>
        <dynamics><damping>20</damping><friction>1</friction></dynamics></axis>
    </joint>
"""

# End sphere
z_end = cable_top_z - (num_segments + 1) * segment_length
sdf += f"""
    <link name="end_sphere">
      <pose>0 0 {z_end:.3f} 0 0 0</pose>
      <inertial><mass>{end_sphere_mass}</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial>
      <visual name="end_sphere_visual">
        <geometry><sphere><radius>{end_sphere_radius}</radius></sphere></geometry>
      </visual>
      <collision name="end_sphere_collision">
        <geometry><sphere><radius>{end_sphere_radius}</radius></sphere></geometry>
      </collision>
    </link>

    <joint name="joint_end_sphere" type="revolute">
      <parent>segment_{num_segments}_cylinder</parent>
      <child>end_sphere</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-1.57</lower><upper>1.57</upper></limit></axis>
    </joint>

    <plugin name="gz::sim::systems::OdometryPublisher" filename="libgz-sim-odometry-publisher-system.so">
      <link_name>end_sphere</link_name>
      <topic>/end_sphere/odom</topic>
      <publish_pose>true</publish_pose>
      <publish_velocity>true</publish_velocity>
    </plugin>

  </model>
</sdf>
"""

# Write to file
with open("model.sdf", "w") as f:
    f.write(sdf)

print("SDF model generated as model.sdf")
