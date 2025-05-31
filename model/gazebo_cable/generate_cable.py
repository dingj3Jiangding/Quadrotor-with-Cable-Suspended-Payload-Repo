num_segments = 30
segment_length = 0.01
base_z = -0.2
damping = 20
friction = 1
sphere_radius = 0.003
cylinder_radius = 0.003
sphere_mass = 0.00005
cylinder_mass = 0.0001

# Start the SDF template
sdf_template = f"""
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="gazebo_cable">
    <static>false</static>
    
    <!-- Base Link -->
    <link name="cable_base_link">
      <pose>0 0 {base_z} 0 0 0</pose>
      <visual name="base_link_visual">
        <geometry>
          <box>
            <size>0.04 0.04 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="base_link_collision">
        <geometry>
          <box>
            <size>0.04 0.04 0.02</size>
          </box>
        </geometry>
      </collision>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.01</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <self_collide>true</self_collide>
    </link>
"""

# Loop to generate segments
for i in range(1, num_segments + 1):
    z_pos = base_z - i * segment_length

    # Sphere as intermediary between segments
    sdf_template += f"""

    <!-- Joint for Previous Cylinder to Sphere -->
    <joint name="segment_{i}_joint" type="revolute">
      <parent>{"cable_base_link" if i == 1 else f"segment_{i-1}_cylinder"}</parent>
      <child>segment_{i}_sphere</child>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-3.14</lower>
          <upper>3.14</upper>
          <effort>10</effort>
          <velocity>3</velocity>
        </limit>
      </axis>
      <dynamics>
        <damping>{damping}</damping>
        <friction>{friction}</friction>
      </dynamics>
    </joint>


    <!-- Sphere {i} -->
    <link name="segment_{i}_sphere">
      <pose>0 0 {z_pos} 0 0 0</pose>
      <visual name="segment_{i}_sphere_visual">
        <geometry>
          <sphere>
            <radius>{sphere_radius}</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{i}_sphere_collision">
        <geometry>
          <sphere>
            <radius>{sphere_radius}</radius>
          </sphere>
        </geometry>
      </collision>
      <inertial>
        <mass>{sphere_mass}</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <self_collide>true</self_collide>
    </link>

    <!-- Joint for Sphere to Cylinder -->
    <joint name="segment_{i}_sphere_to_cylinder" type="revolute">
      <parent>segment_{i}_sphere</parent>
      <child>segment_{i}_cylinder</child>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-3.14</lower>
          <upper>3.14</upper>
          <effort>10</effort>
          <velocity>3</velocity>
        </limit>
        <dynamics>
          <damping>{damping}</damping>
          <friction>{friction}</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Cylinder Segment {i} -->
    <link name="segment_{i}_cylinder">
      <pose>0 0 {z_pos - segment_length/2} 0 0 0</pose>
      <visual name="segment_{i}_cylinder_visual">
        <geometry>
          <cylinder>
            <radius>{cylinder_radius}</radius>
            <length>{segment_length}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0.8 1</ambient>
          <diffuse>0 0 0.8 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
      <collision name="segment_{i}_collision">
        <geometry>
          <cylinder>
            <radius>{cylinder_radius}</radius>
            <length>{segment_length}</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>{cylinder_mass}</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <self_collide>true</self_collide>
    </link>

    """

# Add the end sphere after the last segment
sdf_template += f"""
    <!-- End Sphere -->
    <link name="end_sphere">
      <pose>0 0 {z_pos - segment_length} 0 0 0</pose>
      <visual name="end_sphere_visual">
        <geometry>
          <sphere>
            <radius>0.015</radius>
          </sphere>
        </geometry>
      </visual>
      <collision name="end_sphere_collision">
        <geometry>
          <sphere>
            <radius>0.015</radius>
          </sphere>
        </geometry>
      </collision>
      <inertial>
        <mass>0.08</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
    </link>

    <!-- Joint to End Sphere -->
    <joint name="joint_end_sphere" type="revolute">
      <parent>segment_{num_segments}_cylinder</parent>
      <child>end_sphere</child>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
        </limit>
      </axis>
    </joint>
"""

# Close model tag
sdf_template += """
  </model>
</sdf>
"""

# Write to file
with open("model.sdf", "w") as f:
    f.write(sdf_template)

print("SDF file generated: model.sdf")
