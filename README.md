cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/Users/jisangyou/Desktop/raisim_ws/raisimLib/raisim/mac

cmake .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_PREFIX_PATH="/home/jakob/Desktop/ME491/raisimLib/thirdParty/Eigen3;/home/jakob/Desktop/ME491/raisimLib/raisim/linux"
make -j

 hip abduction/adduction (HAA) joint, the hip flexion/extension (HFE) joint, and the knee flex- ion/extension joint (KFE)


<!-- base link -->
<link name="base">

<!-- joint base HAA -->
<joint name="base_LH_HAA" type="fixed">
    <parent link="base"/>
    <child link="LH_HAA"/>
    <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
</joint>

//REV
<!-- joint Drive output -->
<joint name="LH_HAA" type="revolute">
    <parent link="LH_HAA"/>
    <child link="LH_HIP"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="-1 0 0"/>
    <limit effort="80.0" lower="-0.72" upper="0.49" velocity="7.5"/>
    <dynamics damping="0.0" friction="0.0"/>
</joint>

<!-- joint HAA hip -->
<joint name="LH_HIP_LH_hip_fixed" type="fixed">
    <parent link="LH_HIP"/>
    <child link="LH_hip_fixed"/>
    <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
</joint>

<!-- joint hip HFE -->
<joint name="LH_hip_fixed_LH_HFE" type="fixed">
    <parent link="LH_hip_fixed"/>
    <child link="LH_HFE"/>
    <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
</joint>

//REV
<!-- joint Drive output -->
<joint name="LH_HFE" type="revolute">
    <parent link="LH_HFE"/>
    <child link="LH_THIGH"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="80.0" lower="-9.42477796077" upper="9.42477796077" velocity="7.5"/>
    <dynamics damping="0.0" friction="0.0"/>
</joint>

<!-- joint HFE thigh -->
<joint name="LH_THIGH_LH_thigh_fixed" type="fixed">
    <parent link="LH_THIGH"/>
    <child link="LH_thigh_fixed"/>
    <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
</joint>

<!-- joint thigh KFE -->
<joint name="LH_thigh_fixed_LH_KFE" type="fixed">
    <parent link="LH_thigh_fixed"/>
    <child link="LH_KFE"/>
    <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
</joint>

//REV
<!-- joint Drive output -->
<joint name="LH_KFE" type="revolute">
    <parent link="LH_KFE"/>
    <child link="LH_SHANK"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="80.0" lower="-9.42477796077" upper="9.42477796077" velocity="7.5"/>
    <dynamics damping="0.0" friction="0.0"/>
</joint>

<!-- joint KFE shank -->
<joint name="LH_shank_LH_shank_fixed" type="fixed">
    <parent link="LH_SHANK"/>
    <child link="LH_shank_fixed"/>
    <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
</joint>

<!-- joint shank foot -->
<joint name="LH_shank_fixed_LH_FOOT" type="fixed">
    <parent link="LH_shank_fixed"/>
    <child link="LH_FOOT"/>
    <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
</joint>





//EXCERCISE 3

<?xml version="1.0" encoding="utf-8"?>
<!-- =================================================================================== -->
<!--   Copyright 2020 ANYbotics, https://www.anybotics.com                               -->
<!-- =================================================================================== -->
<!-- This file contains the description of the ANYmal C robot. -->
<robot name="anymal">
    <!-- Base link -->
    <link name="base">
    <!-- Fixed joint to add dummy inertia link -->
    <joint name="base_to_base_inertia" type="fixed">
        <parent link="base"/>
        <child link="base_inertia"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>
    <!-- Dummy inertia link, because KDL cannot have inertia on the base link -->
    <link name="base_inertia">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.018 -0.002 0.024"/>
            <mass value="6.222"/>
            <inertia ixx="0.017938806" ixy="0.00387963" ixz="0.001500772" iyy="0.370887745" iyz="6.8963e-05" izz="0.372497653"/>
        </inertial>
    </link>
    <!-- Fixed joint base face -->
    <joint name="base_face_front" type="fixed">
        <parent link="base"/>
        <child link="face_front"/>
        <origin rpy="0 0 -0.0" xyz="0.4145 0 0"/>
    </joint>
    <!-- Shell link -->
    <link name="face_front">
        <inertial>
            <origin rpy="0 0 0" xyz="0.042 -0.001 0.004"/>
            <mass value="0.73"/>
            <inertia ixx="0.005238611" ixy="1.7609e-05" ixz="7.2167e-05" iyy="0.002643098" iyz="1.9548e-05" izz="0.004325938"/>
        </inertial>
    </link>
    <!-- Fixed joint base face -->
    <joint name="base_face_rear" type="fixed">
        <parent link="base"/>
        <child link="face_rear"/>
        <origin rpy="0 0 3.14159265359" xyz="-0.4145 0 0"/>
    </joint>
    <!-- Shell link -->
    <link name="face_rear">
        <inertial>
            <origin rpy="0 0 0" xyz="0.042 -0.001 0.004"/>
            <mass value="0.73"/>
            <inertia ixx="0.005238611" ixy="1.7609e-05" ixz="7.2167e-05" iyy="0.002643098" iyz="1.9548e-05" izz="0.004325938"/>
        </inertial>
    </link>
    <!-- Fixed joint base battery -->
    <joint name="base_battery" type="fixed">
        <parent link="base"/>
        <child link="battery"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>
    <!-- Shell link -->
    <link name="battery">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.00067 -0.00023 -0.03362"/>
            <mass value="5.53425"/>
            <inertia ixx="0.00749474794" ixy="0.00016686282" ixz="7.82763e-05" iyy="0.0722338913" iyz="1.42902e-06" izz="0.07482717535"/>
        </inertial>
    </link>
    <!-- Fixed joint to add docking  -->
    <joint name="base_to_docking_hatch_cover" type="fixed">
        <parent link="base"/>
        <child link="docking_hatch_cover"/>
        <origin rpy="0 0 0" xyz="0.343 0.0 -0.07"/>
    </joint>
    <!-- Hatch cover link -->
    <link name="docking_hatch_cover">
        <!--  TODO(fgiraldez): add visual    -->
        <inertial>
            <origin rpy="0 0 0" xyz="-0.003 0.0 0.005"/>
            <mass value="0.065"/>
            <inertia ixx="0.00063283" ixy="0.0" ixz="3.45e-07" iyy="0.00110971" iyz="0.0" izz="0.00171883"/>
        </inertial>
    </link>
    <!-- parent to cage joint, located between mounting plate on trunk and the cage -->
    <joint name="base_to_lidar_cage" type="fixed">
        <parent link="base"/>
        <child link="lidar_cage"/>
        <origin rpy="0 0 0" xyz="-0.364 0.0 0.0735"/>
    </joint>
    <!-- Velodyne cage link -->
    <link name="lidar_cage">
    <!-- parent to sensor joint -->
    <joint name="lidar_cage_to_lidar" type="fixed">
        <parent link="lidar_cage"/>
        <child link="lidar"/>
        <origin rpy="0.0 0.0 -1.57079632679" xyz="0.0 0.0 0.0687"/>
    </joint>
    <!-- Velodyne sensor link -->
    <link name="lidar">
        <inertial>
            <origin rpy="0.0 0.0 0.0" xyz="-0.012 0.001 -0.008"/>
            <mass value="0.695"/>
            <inertia ixx="0.000846765" ixy="6.9565e-05" ixz="0.00027111" iyy="0.001367583" iyz="5.8984e-05" izz="0.001363673"/>
        </inertial>
    </link>
    <!-- joint base HAA -->
    <joint name="base_LF_HAA" type="fixed">
        <parent link="base"/>
        <child link="LF_HAA"/>
        <origin rpy="2.61799387799 0 0.0" xyz="0.2999 0.104 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="LF_HAA">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LF_HAA" type="revolute">
        <parent link="LF_HAA"/>
        <child link="LF_HIP"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LF_HIP">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HAA hip -->
    <joint name="LF_HIP_LF_hip_fixed" type="fixed">
        <parent link="LF_HIP"/>
        <child link="LF_hip_fixed"/>
        <origin rpy="-2.61799387799 0 0.0" xyz="0 0 0"/>
    </joint>
    <!-- Hip link -->
    <link name="LF_hip_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.048 0.008 -0.003"/>
            <mass value="0.74"/>
            <inertia ixx="0.001393106" ixy="8.4012e-05" ixz="2.3378e-05" iyy="0.003798579" iyz="7.1319e-05" izz="0.003897509"/>
        </inertial>
    </link>
    <!-- joint hip HFE -->
    <joint name="LF_hip_fixed_LF_HFE" type="fixed">
        <parent link="LF_hip_fixed"/>
        <child link="LF_HFE"/>
        <origin rpy="0 0 1.57079632679" xyz="0.0599 0.08381 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="LF_HFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LF_HFE" type="revolute">
        <parent link="LF_HFE"/>
        <child link="LF_THIGH"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LF_THIGH">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HFE thigh -->
    <joint name="LF_THIGH_LF_thigh_fixed" type="fixed">
        <parent link="LF_THIGH"/>
        <child link="LF_thigh_fixed"/>
        <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Thigh link -->
    <link name="LF_thigh_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.0 0.018 -0.169"/>
            <mass value="1.03"/>
            <inertia ixx="0.018644469" ixy="5.2e-08" ixz="1.0157e-05" iyy="0.019312599" iyz="0.002520077" izz="0.002838361"/>
        </inertial>
    </link>
    <!-- joint thigh KFE -->
    <joint name="LF_thigh_fixed_LF_KFE" type="fixed">
        <parent link="LF_thigh_fixed"/>
        <child link="LF_KFE"/>
        <origin rpy="0 0 1.57079632679" xyz="0.0 0.1003 -0.285"/>
    </joint>
    <!-- Drive link -->
    <link name="LF_KFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LF_KFE" type="revolute">
        <parent link="LF_KFE"/>
        <child link="LF_SHANK"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LF_SHANK">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint KFE shank -->
    <joint name="LF_shank_LF_shank_fixed" type="fixed">
        <parent link="LF_SHANK"/>
        <child link="LF_shank_fixed"/>
        <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Shank link -->
    <link name="LF_shank_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.03463 0.00688 0.00098"/>
            <mass value="0.33742"/>
            <inertia ixx="0.00032748005" ixy="2.142561e-05" ixz="1.33942e-05" iyy="0.00110974122" iyz="7.601e-08" izz="0.00089388521"/>
        </inertial>
    </link>
    <!-- Leg configurations: xx (knees bent inwards), xo (knees bent backwards) -->
    <!-- joint shank foot -->
    <joint name="LF_shank_fixed_LF_FOOT" type="fixed">
        <parent link="LF_shank_fixed"/>
        <child link="LF_FOOT"/>
        <origin rpy="0 0 0" xyz="0.08795 0.01305 -0.33797"/>
    </joint>
    <!-- Foot link -->
    <link name="LF_FOOT">
        <inertial>
            <origin rpy="0 0 0" xyz="0.00948 -0.00948 0.1468"/>
            <mass value="0.25"/>
            <inertia ixx="0.00317174097" ixy="2.63048e-06" ixz="6.815581e-05" iyy="0.00317174092" iyz="6.815583e-05" izz="8.319196e-05"/>
        </inertial>
    </link>
    <!-- joint base HAA -->
    <joint name="base_RF_HAA" type="fixed">
        <parent link="base"/>
        <child link="RF_HAA"/>
        <origin rpy="-2.61799387799 0 0.0" xyz="0.2999 -0.104 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="RF_HAA">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RF_HAA" type="revolute">
        <parent link="RF_HAA"/>
        <child link="RF_HIP"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RF_HIP">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HAA hip -->
    <joint name="RF_HIP_RF_hip_fixed" type="fixed">
        <parent link="RF_HIP"/>
        <child link="RF_hip_fixed"/>
        <origin rpy="2.61799387799 0 0.0" xyz="0 0 0"/>
    </joint>
    <!-- Hip link -->
    <link name="RF_hip_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.048 -0.008 -0.003"/>
            <mass value="0.74"/>
            <inertia ixx="0.001393106" ixy="-8.4012e-05" ixz="2.3378e-05" iyy="0.003798579" iyz="-7.1319e-05" izz="0.003897509"/>
        </inertial>
    </link>
    <!-- joint hip HFE -->
    <joint name="RF_hip_fixed_RF_HFE" type="fixed">
        <parent link="RF_hip_fixed"/>
        <child link="RF_HFE"/>
        <origin rpy="0 0 -1.57079632679" xyz="0.0599 -0.08381 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="RF_HFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RF_HFE" type="revolute">
        <parent link="RF_HFE"/>
        <child link="RF_THIGH"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RF_THIGH">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HFE thigh -->
    <joint name="RF_THIGH_RF_thigh_fixed" type="fixed">
        <parent link="RF_THIGH"/>
        <child link="RF_thigh_fixed"/>
        <origin rpy="0 0 1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Thigh link -->
    <link name="RF_thigh_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.0 -0.018 -0.169"/>
            <mass value="1.03"/>
            <inertia ixx="0.018644469" ixy="-5.2e-08" ixz="1.0157e-05" iyy="0.019312599" iyz="-0.002520077" izz="0.002838361"/>
        </inertial>
    </link>
    <!-- joint thigh KFE -->
    <joint name="RF_thigh_fixed_RF_KFE" type="fixed">
        <parent link="RF_thigh_fixed"/>
        <child link="RF_KFE"/>
        <origin rpy="0 0 -1.57079632679" xyz="0.0 -0.1003 -0.285"/>
    </joint>
    <!-- Drive link -->
    <link name="RF_KFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RF_KFE" type="revolute">
        <parent link="RF_KFE"/>
        <child link="RF_SHANK"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RF_SHANK">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint KFE shank -->
    <joint name="RF_shank_RF_shank_fixed" type="fixed">
        <parent link="RF_SHANK"/>
        <child link="RF_shank_fixed"/>
        <origin rpy="0 0 1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Shank link -->
    <link name="RF_shank_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="0.03463 -0.00688 0.00098"/>
            <mass value="0.33742"/>
            <inertia ixx="0.00032748005" ixy="-2.142561e-05" ixz="1.33942e-05" iyy="0.00110974122" iyz="-7.601e-08" izz="0.00089388521"/>
        </inertial>
    </link>
    <!-- Leg configurations: xx (knees bent inwards), xo (knees bent backwards) -->
    <!-- joint shank foot -->
    <joint name="RF_shank_fixed_RF_FOOT" type="fixed">
        <parent link="RF_shank_fixed"/>
        <child link="RF_FOOT"/>
        <origin rpy="0 0 0" xyz="0.08795 -0.01305 -0.33797"/>
    </joint>
    <!-- Foot link -->
    <link name="RF_FOOT">
        <inertial>
            <origin rpy="0 0 0" xyz="0.00948 0.00948 0.1468"/>
            <mass value="0.25"/>
            <inertia ixx="0.00317174097" ixy="-2.63048e-06" ixz="6.815581e-05" iyy="0.00317174092" iyz="-6.815583e-05" izz="8.319196e-05"/>
        </inertial>
    </link>
    <!-- joint base HAA -->
    <joint name="base_LH_HAA" type="fixed">
        <parent link="base"/>
        <child link="LH_HAA"/>
        <origin rpy="-2.61799387799 0 -3.14159265359" xyz="-0.2999 0.104 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="LH_HAA">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LH_HAA" type="revolute">
        <parent link="LH_HAA"/>
        <child link="LH_HIP"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LH_HIP">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HAA hip -->
    <joint name="LH_HIP_LH_hip_fixed" type="fixed">
        <parent link="LH_HIP"/>
        <child link="LH_hip_fixed"/>
        <origin rpy="-2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
    </joint>
    <!-- Hip link -->
    <link name="LH_hip_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.048 0.008 -0.003"/>
            <mass value="0.74"/>
            <inertia ixx="0.001393106" ixy="-8.4012e-05" ixz="-2.3378e-05" iyy="0.003798579" iyz="7.1319e-05" izz="0.003897509"/>
        </inertial>
    </link>
    <!-- joint hip HFE -->
    <joint name="LH_hip_fixed_LH_HFE" type="fixed">
        <parent link="LH_hip_fixed"/>
        <child link="LH_HFE"/>
        <origin rpy="0 0 1.57079632679" xyz="-0.0599 0.08381 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="LH_HFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LH_HFE" type="revolute">
        <parent link="LH_HFE"/>
        <child link="LH_THIGH"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LH_THIGH">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HFE thigh -->
    <joint name="LH_THIGH_LH_thigh_fixed" type="fixed">
        <parent link="LH_THIGH"/>
        <child link="LH_thigh_fixed"/>
        <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Thigh link -->
    <link name="LH_thigh_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.0 0.018 -0.169"/>
            <mass value="1.03"/>
            <inertia ixx="0.018644469" ixy="-5.2e-08" ixz="-1.0157e-05" iyy="0.019312599" iyz="0.002520077" izz="0.002838361"/>
        </inertial>
    </link>
    <!-- joint thigh KFE -->
    <joint name="LH_thigh_fixed_LH_KFE" type="fixed">
        <parent link="LH_thigh_fixed"/>
        <child link="LH_KFE"/>
        <origin rpy="0 0 1.57079632679" xyz="-0.0 0.1003 -0.285"/>
    </joint>
    <!-- Drive link -->
    <link name="LH_KFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="LH_KFE" type="revolute">
        <parent link="LH_KFE"/>
        <child link="LH_SHANK"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="LH_SHANK">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint KFE shank -->
    <joint name="LH_shank_LH_shank_fixed" type="fixed">
        <parent link="LH_SHANK"/>
        <child link="LH_shank_fixed"/>
        <origin rpy="0 0 -1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Shank link -->
    <link name="LH_shank_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.03463 0.00688 0.00098"/>
            <mass value="0.33742"/>
            <inertia ixx="0.00032748005" ixy="-2.142561e-05" ixz="-1.33942e-05" iyy="0.00110974122" iyz="7.601e-08" izz="0.00089388521"/>
        </inertial>
    </link>
    <!-- Leg configurations: xx (knees bent inwards), xo (knees bent backwards) -->
    <!-- joint shank foot -->
    <joint name="LH_shank_fixed_LH_FOOT" type="fixed">
        <parent link="LH_shank_fixed"/>
        <child link="LH_FOOT"/>
        <origin rpy="0 0 0" xyz="-0.08795 0.01305 -0.33797"/>
    </joint>
    <!-- Foot link -->
    <link name="LH_FOOT">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.00948 -0.00948 0.1468"/>
            <mass value="0.25"/>
            <inertia ixx="0.00317174097" ixy="-2.63048e-06" ixz="-6.815581e-05" iyy="0.00317174092" iyz="6.815583e-05" izz="8.319196e-05"/>
        </inertial>
    </link>
    <!-- joint base HAA -->
    <joint name="base_RH_HAA" type="fixed">
        <parent link="base"/>
        <child link="RH_HAA"/>
        <origin rpy="2.61799387799 0 -3.14159265359" xyz="-0.2999 -0.104 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="RH_HAA">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RH_HAA" type="revolute">
        <parent link="RH_HAA"/>
        <child link="RH_HIP"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RH_HIP">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HAA hip -->
    <joint name="RH_HIP_RH_hip_fixed" type="fixed">
        <parent link="RH_HIP"/>
        <child link="RH_hip_fixed"/>
        <origin rpy="2.61799387799 0 -3.14159265359" xyz="0 0 0"/>
    </joint>
    <!-- Hip link -->
    <link name="RH_hip_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.048 -0.008 -0.003"/>
            <mass value="0.74"/>
            <inertia ixx="0.001393106" ixy="8.4012e-05" ixz="-2.3378e-05" iyy="0.003798579" iyz="-7.1319e-05" izz="0.003897509"/>
        </inertial>
    </link>
    <!-- joint hip HFE -->
    <joint name="RH_hip_fixed_RH_HFE" type="fixed">
        <parent link="RH_hip_fixed"/>
        <child link="RH_HFE"/>
        <origin rpy="0 0 -1.57079632679" xyz="-0.0599 -0.08381 0.0"/>
    </joint>
    <!-- Drive link -->
    <link name="RH_HFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RH_HFE" type="revolute">
        <parent link="RH_HFE"/>
        <child link="RH_THIGH"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RH_THIGH">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint HFE thigh -->
    <joint name="RH_THIGH_RH_thigh_fixed" type="fixed">
        <parent link="RH_THIGH"/>
        <child link="RH_thigh_fixed"/>
        <origin rpy="0 0 1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Thigh link -->
    <link name="RH_thigh_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.0 -0.018 -0.169"/>
            <mass value="1.03"/>
            <inertia ixx="0.018644469" ixy="5.2e-08" ixz="-1.0157e-05" iyy="0.019312599" iyz="-0.002520077" izz="0.002838361"/>
        </inertial>
    </link>
    <!-- joint thigh KFE -->
    <joint name="RH_thigh_fixed_RH_KFE" type="fixed">
        <parent link="RH_thigh_fixed"/>
        <child link="RH_KFE"/>
        <origin rpy="0 0 -1.57079632679" xyz="-0.0 -0.1003 -0.285"/>
    </joint>
    <!-- Drive link -->
    <link name="RH_KFE">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.063 7e-05 0.00046"/>
            <mass value="2.04"/>
            <inertia ixx="0.001053013" ixy="4.527e-05" ixz="8.855e-05" iyy="0.001805509" iyz="9.909e-05" izz="0.001765827"/>
        </inertial>
    </link>
    <!-- joint Drive output -->
    <joint name="RH_KFE" type="revolute">
        <parent link="RH_KFE"/>
        <child link="RH_SHANK"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <axis xyz="-1 0 0"/>
    </joint>
    <!-- Drive output link -->
    <link name="RH_SHANK">
        <inertial>
            <origin rpy="0 0 0" xyz="0 0 0"/>
            <mass value="0.001"/>
            <inertia ixx="0.000001" ixy="0.0" ixz="0.0" iyy="0.000001" iyz="0.0" izz="0.000001"/>
        </inertial>
    </link>
    <!-- joint KFE shank -->
    <joint name="RH_shank_RH_shank_fixed" type="fixed">
        <parent link="RH_SHANK"/>
        <child link="RH_shank_fixed"/>
        <origin rpy="0 0 1.57079632679" xyz="0 0 0"/>
    </joint>
    <!-- Shank link -->
    <link name="RH_shank_fixed">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.03463 -0.00688 0.00098"/>
            <mass value="0.33742"/>
            <inertia ixx="0.00032748005" ixy="2.142561e-05" ixz="-1.33942e-05" iyy="0.00110974122" iyz="-7.601e-08" izz="0.00089388521"/>
        </inertial>
    </link>
    <!-- Leg configurations: xx (knees bent inwards), xo (knees bent backwards) -->
    <!-- joint shank foot -->
    <joint name="RH_shank_fixed_RH_FOOT" type="fixed">
        <parent link="RH_shank_fixed"/>
        <child link="RH_FOOT"/>
        <origin rpy="0 0 0" xyz="-0.08795 -0.01305 -0.33797"/>
    </joint>
    <!-- Foot link -->
    <link name="RH_FOOT">
        <inertial>
            <origin rpy="0 0 0" xyz="-0.00948 0.00948 0.1468"/>
            <mass value="0.25"/>
            <inertia ixx="0.00317174097" ixy="2.63048e-06" ixz="-6.815581e-05" iyy="0.00317174092" iyz="-6.815583e-05" izz="8.319196e-05"/>
        </inertial>
    </link>
    <!-- Fixed joint base hatch -->
    <joint name="base_hatch" type="fixed">
        <parent link="base"/>
        <child link="hatch"/>
        <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
    </joint>
    <!-- Hatch link -->
    <link name="hatch">
        <inertial>
            <origin rpy="0 0 0" xyz="0.116 0.0 0.0758"/>
            <mass value="0.142"/>
            <inertia ixx="0.001" ixy="0.001" ixz="0.001" iyy="0.001" iyz="0.001" izz="0.001"/>
        </inertial>
    </link>
</robot>