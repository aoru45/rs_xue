# 跨平台 em4 Python 接口

## 安装

在项目根目录执行：

```bash
export VCPKG_ROOT=/path/to/vcpkg
vcpkg install
pip install -e .
```

这个仓库根目录自带 [vcpkg.json](/Volumes/T7/repo/rs_xue/vcpkg.json)，`setup.py` 会优先读取 `VCPKG_ROOT` 并把 `vcpkg` toolchain 传给 CMake。

如果你已经手动配置了 toolchain，也可以直接设置：

```bash
export CMAKE_TOOLCHAIN_FILE="$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake"
pip install -e .
```

## 使用

```python
import rs_xue

client = rs_xue.Client()
reader = rs_xue.PcapReader()
```
