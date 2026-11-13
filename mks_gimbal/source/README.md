# 源代码与重新生成

普通生成复用 `reference/reference_evidence.json`，不重复测量 STEP。`source/lib/` 包含原样复用的 MIT 3d-print 导出/检查工具；只需要 numpy、trimesh、manifold3d。依赖已在本机独立 venv 中验证，不需要改 OpenCode 模型或 Provider 配置。

在交付目录执行 PowerShell：

```powershell
$env:PYTHONUTF8='1'
$py='C:\Users\HLi\.config\opencode\skill-envs\3d-print\Scripts\python.exe'
& $py 'source\build_gimbal.py'
& $py 'source\build_fit_tests.py'
```

重新生成的总装/打印模型写入交付目录下新建的 `output/`，不会覆盖根目录已经交付的模型。只调整个别零件且已有一次完整 `output/` 后，可用：

```powershell
& $py 'source\build_gimbal.py' --parts '05_Pan_25T_Hub,11_Tilt_25T_Hub'
```

未改的零件复用已通过验证的实际网格，仍重新检查总装，避免另写同类 probe。若改变影响其他零件的参数，应完整生成。

Blender 4.5.16 LTS 已用于本机交付：

```powershell
$blender='C:\Users\HLi\AppData\Local\Temp\opencode\gimbal-blender\blender-4.5.16-windows-x64\blender.exe'
& $blender -b --factory-startup --python 'source\blender_export.py'
& $py 'source\make_viewer.py'
```

Blender 导出会拒绝带未解决装配错误、或与当前 manifest 不匹配的验证报告。生成脚本保留精确坐标、原始参考哈希和验证边界。`measure_reference.py` 是最初 CAD 取证脚本，需要 cadquery / OpenCascade，仅在参考来源变化或出现矛盾时使用，不是常规生成步骤。

当前配合值为试配默认值，不是已实物确认值。齿面放大量在 `geometry.py` 的 `spline_cs` / `spline_tool` / `spline_lead`；铜套孔在 `insert_tool`；轴承座、轴颈及装配位置在 `build_gimbal.py`。修改后要同步 manifest 中的配合说明，并重新检查关联零件。不要缩放用户的原始舵机网格。
