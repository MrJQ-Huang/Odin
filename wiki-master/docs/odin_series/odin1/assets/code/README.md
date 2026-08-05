# map_to_ply

将 `loop_map.bin`（MAPV0001 格式）转换为 PLY 点云文件。

## 使用方法

```bash
./map_to_ply <input.bin> [output.ply]
```

- `input.bin` — 输入的 `loop_map.bin` 文件
- `output.ply` — 输出的 PLY 文件（可选，默认与输入同名，扩展名改为 `.ply`）

## 示例

```bash
# 自动命名输出文件
./map_to_ply loop_map.bin
# 生成 loop_map.ply

# 指定输出文件名
./map_to_ply loop_map.bin my_map.ply
```

## 平台

| 文件 | 平台 |
|------|------|
| `map_to_ply_amd64` | x86_64 (Intel/AMD) |
| `map_to_ply_arm64` | ARM aarch64 (Jetson / 鲲鹏 等) |

基于 ubuntu 20.04 (glibc 2.31) 编译，兼容 ubuntu 20.04 / 22.04 / 24.04。

## 输出格式

默认输出 **binary little-endian PLY**，包含每个点的 x/y/z 坐标（float32）。
