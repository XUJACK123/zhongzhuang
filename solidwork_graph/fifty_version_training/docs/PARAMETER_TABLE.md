# fifty_version_training 参数表

由 `tools/build_training_urdf.py` 生成，数值改动后请重新生成，不要手工编辑。

| 参数 | 数值 | 来源 | 可信度 |
|---|---:|---|---|
| `total_mass_kg` | 23.259630516 | URDF / SW export | mechanical design (not weighed) |
| `wheel_radius_m` | 0.0600 | STL radius about axle | PROVISIONAL - measure the real wheel |
| `thigh_planar_m` | 0.210000 | URDF FK | exact from geometry |
| `shin_planar_m` | 0.250000 | URDF FK | exact from geometry |
| `hip_separation_m` | 0.455396 | URDF FK | exact from geometry |
| `wheel_base_m` | 0.529000 | URDF FK | exact from geometry |
| `spawn_height_m` | 0.474751 | computed | default stance + 10 mm clearance |
| `base_com_forward_m` | see model_parameters | URDF inertial | mechanical design |
| `leg_joint_effort_nm` | 30.0 | SW export | PROVISIONAL - no motor data |
| `leg_joint_velocity_rad_s` | 1000 | SW export | PROVISIONAL |

## 默认关节角（= SW 装配姿态在复旦坐标下的表示）

| joint | q (rad) | 说明 |
|---|---:|---|
| `rf0_Joint` | 2.055422 | leg |
| `rf1_Joint` | -2.559268 | leg |
| `r_wheel_Joint` | 0.000000 | wheel |
| `lf0_Joint` | -2.055422 | leg |
| `lf1_Joint` | 2.559268 | leg |
| `l_wheel_Joint` | 0.000000 | wheel |

## 关节限位（provisional）

| joint | lower | upper |
|---|---:|---:|
| `rf0_Joint` | 1.4554 | 2.6554 |
| `rf1_Joint` | -3.2593 | -1.8593 |
| `r_wheel_Joint` | -inf | +inf (continuous) |
| `lf0_Joint` | -2.6554 | -1.4554 |
| `lf1_Joint` | 1.8593 | 3.2593 |
| `l_wheel_Joint` | -inf | +inf (continuous) |

## 符号表（positive q 的物理方向）

| joint | axis sign (w.r.t. new +Y) |
|---|---:|
| `rf0_Joint` | +1 |
| `rf1_Joint` | +1 |
| `r_wheel_Joint` | +1 |
| `lf0_Joint` | -1 |
| `lf1_Joint` | -1 |
| `l_wheel_Joint` | +1 |

## 构建校验

```
{
  "xml_parses": true,
  "links": true,
  "joints": true,
  "dof_order": true,
  "mass_preserved": true,
  "inertia_positive_definite": true,
  "mesh_files_exist": true,
  "meshes_unchanged": true,
  "default_angles_inside_limits": true,
  "fudan_formula_fk_match": true,
  "zero_pose_thigh_horizontal": true,
  "zero_pose_shin_vertical": true
}
```

FK vs Fudan 公式最大误差：1.110e-15 m
