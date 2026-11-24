# 6200RS 少重打版 · 源代码与重新生成

普通生成复用 `reference/reference_evidence.json`，不重复测量 STEP。`source/lib/` 包含原样复用的 MIT 3d-print 导出/检查工具；只需要 numpy、trimesh、manifold3d。依赖已在本机独立 venv 中验证，不需要改 OpenCode 模型或 Provider 配置。

所有6200参数集中在 `bearing_design.py`，尺寸证据在 `reference/hardware_evidence.json`。原始MKS表面、坐标与真实25T截面不变。新05/17分体，18的三个实例共用一个打印文件。不要另写重复测量/几何probe。

在交付目录执行 PowerShell：

```powershell
$env:PYTHONUTF8='1'
$py='C:\Users\HLi\.config\opencode\skill-envs\3d-print\Scripts\python.exe'
& $py 'source\build_gimbal.py' --help
& $py 'source\build_fit_tests.py' --help
& $py 'source\build_gimbal.py'
& $py 'source\build_fit_tests.py' --bearings-only
```

重新生成的总装/打印模型写入交付目录下新建的 `output/`，不会覆盖根目录已经交付的模型。只调整个别零件且已有一次完整 `output/` 后，可用：

```powershell
& $py 'source\build_gimbal.py' --parts '17_Pan_25T_Coupler'
```

未改的零件复用已通过验证的实际网格，仍重新检查总装，避免另写同类 probe。若改变影响其他零件的参数，应完整生成。

Blender 4.5.16 LTS 已用于本机交付：

```powershell
$blender='C:\Users\HLi\AppData\Local\Programs\Blender-MCP\blender-4.5.16-windows-x64\blender.exe'
& $blender -b --factory-startup --python 'source\blender_export.py' -- --help
& $blender -b --factory-startup --python-exit-code 1 --python 'source\blender_export.py'
& $py 'source\make_viewer.py'
```

Blender 导出会拒绝带未解决装配错误、或与当前 manifest 不匹配的验证报告。生成脚本保留精确坐标、原始参考哈希和验证边界。`measure_reference.py` 是最初 CAD 取证脚本，需要 cadquery / OpenCascade，仅在参考来源变化或出现矛盾时使用，不是常规生成步骤。

当前配合值为试配默认值，不是已实物确认值。齿面放大量在 `geometry.py` 的 `spline_cs` / `spline_tool` / `spline_lead`；铜套孔在 `insert_tool`；轴承座、轴颈及装配位置在 `build_gimbal.py`。修改后要同步 manifest 中的配合说明，并重新检查关联零件。不要缩放用户的原始舵机网格。

`--repose`只用于把原13从LW1装配位置向左移5 mm，同时保持原STL/3MF字节；首次少重打迁移已完成，不要对已移动的同一output重复使用此选项。普通局部构建从当前manifest复用其已有装配网格。

## 历史LW1工具与当前运动副本

本版保留LW1的底座/斜肋减材结构，但轴承接口和其他六件已经变化。`verify_lightweight.py`是历史两件减材比较工具，不能用它宣称本次其余14件不变或刚度相同。

```powershell
& $py 'source\package_delivery.py' --help
```

`package_delivery.py`核对现版报告绑定、18种打印文件、三颗6200、旧文件实际SHA256和仅8种的重打包；生成BOM/PARTS/FASTENERS及完整文件哈希。不会覆盖旧设计或强制替换已存在的交付目录。它不是实物或FEA验收。

静态 Blender 导出完成后，可在单独的后台 Blender 进程重建动画副本、渲染并读 MP4 元数据：

```powershell
& $blender -b --factory-startup --python 'source\blender_motion.py'
& $blender --disable-autoexec -b 'output\MKS_PanTilt_motion.blend' -a
& $py 'source\inspect_mp4.py' 'output\PanTilt_motion_preview.mp4'
```

此流程在 `output/` 中工作，不覆盖根目录交付文件，不改变静态总装几何。视频生成后仍要确认使用的是本次输出版本。
