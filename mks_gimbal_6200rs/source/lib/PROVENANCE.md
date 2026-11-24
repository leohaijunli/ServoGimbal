# 复用工具来源

`generate_model.py`、`validate_mesh.py` 来自已安装的 3d-print skill，原样复制，不修改上游文件。

- 原项目：https://github.com/EdwinjJ1/3d-print-skill
- 固定来源提交：`248576d866b2c5d6aa9c236b146d64d3a2063271`
- 上游 README 声明 MIT 许可。
- 只调用其 `export_stl`、`export_3mf` 和 `validate_mesh`，不调用早先失败的 revolve/chamfer helper，也不使用自动修网格。

MIT License

Copyright (c) 3d-print-skill authors and contributors.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
