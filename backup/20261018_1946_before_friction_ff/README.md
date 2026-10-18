# 摩擦补偿前馈修改前的代码快照

- 时间：2026-10-18 19:46（Asia/Shanghai）
- 内容：加入摩擦补偿前馈之前的 `Tasks` 源码、`config.h` 和 Ozone 用户配置。
- 状态：该版本已通过 CMake Debug 编译，ELF 位于 `build/project.elf`。

## 包含文件

- `ControlTask.cpp`
- `ControlTask.h`
- `config.h`
- `Gm6020.cpp`
- `Gm6020.h`
- `Pid.cpp`
- `Pid.h`
- `debug.jdebug.user`

## 恢复方法

把本目录下的 `.cpp` / `.h` 文件复制回工程的 `Tasks/` 目录，把 `debug.jdebug.user`
复制回工程根目录，然后重新编译：

```powershell
cmake --build --preset Debug
```

注意：本快照不包含 `Core`、`Drivers` 和 `build` 目录，恢复时不要覆盖这些目录。
