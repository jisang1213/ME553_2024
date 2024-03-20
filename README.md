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