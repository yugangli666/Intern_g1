# Flash-Attn Thor sm_110 支持修复 - 已完成 ✅

## 最终状态
**成功** - flash-attn 2.8.3.post1 已编译、安装并验证，完整支持 NVIDIA Thor sm_110

## 问题背景
- GPU: NVIDIA Thor，计算能力 11.0 (sm_110)
- CUDA: 13.0
- PyTorch: 2.8.0a0+34c6371d24.nv25.08
- 原 flash-attn 2.7.4.post1 只含 sm_80/90/100/120，缺 sm_110 且无 PTX fallback
- 运行时报错: `CUDA error: no kernel image is available for execution on the device`

## 解决方案与执行结果

### 1. 源码准备 ✅
- 从 GitHub 克隆 flash-attention v2.8.3.post1
- 路径: `/tmp/flash_attn_src_inspect/flash-attention`

### 2. setup.py 补丁 ✅
两处修改：
- 第 70 行 - 默认架构列表添加 "110"：
  ```python
  return os.getenv("FLASH_ATTN_CUDA_ARCHS", "80;90;100;110;120").split(";")
  ```
- 第 189-191 行 - 添加 sm_110 gencode（CUDA >= 13.0）：
  ```python
  if bare_metal_version >= Version("13.0") and "110" in cuda_archs():
      cc_flag.append("-gencode")
      cc_flag.append("arch=compute_110,code=sm_110")
  ```

### 3. 编译 ✅
- 环境变量:
  - `FLASH_ATTN_CUDA_ARCHS=110`（仅编译 sm_110，缩短构建时间）
  - `MAX_JOBS=4`, `NVCC_THREADS=4`
  - `FLASH_ATTENTION_FORCE_BUILD=TRUE`
  - `LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib:...`
- 耗时: 约 40 分钟（12:45 开始，13:21 完成）
- 产物: `dist/flash_attn-2.8.3.post1-cp312-cp312-linux_aarch64.whl` (64MB)

### 4. 安装 ✅
```bash
pip3 install --force-reinstall --no-deps \
  /tmp/flash_attn_src_inspect/flash-attention/dist/flash_attn-2.8.3.post1-cp312-cp312-linux_aarch64.whl
```
- 2.7.4.post1 已卸载
- 2.8.3.post1 已安装

### 5. 验证 ✅

| 验证项 | 结果 |
|--------|------|
| 包版本 | flash-attn 2.8.3.post1 |
| SO 架构 | 仅 sm_110（`cuobjdump --list-elf` 确认）|
| 模块导入 | `flash_attn`, `flash_attn_2_cuda` 导入成功 |
| flash_attn_func | ✅ 输出形状正确，无 NaN/Inf |
| flash_attn_varlen_func | ✅ 输出正确 |
| InternVLAN1AsyncAgent 导入 | ✅ 成功 |
| 推理服务启动 | ✅ `[INFO] flash-attn 2.8.3.post1 已启用 (attn_implementation=flash_attention_2)` |
| 无 eager 回退 | ✅ 未设置 INTERNNAV_ALLOW_EAGER_ATTN |
| /health | ✅ status ok, cuda:0 |
| /eval_dual 推理 | ✅ 返回 `{'discrete_action': [2, 2, 2, 2]}` |
| no kernel image 错误 | ✅ 已消除 |

## 关键日志证据
```
[INFO] flash-attn 2.8.3.post1 已启用 (attn_implementation=flash_attention_2)
Loading checkpoint shards: 100%|██████████| 4/4 [00:14<00:00,  3.68s/it]
 * Running on http://127.0.0.1:5801
{"device":"cuda:0","model_path":"/workspace/model_vln","status":"ok"}
✅ 推理成功！模型输出: {'discrete_action': [2, 2, 2, 2]}
```

## 后续可执行

推理服务现已正常运行（flash-attn 模式）。可直接运行无运动 VLN 测试：
```bash
bash scripts/realworld/start_no_motion_test.sh
```
测试脚本默认 `INTERNNAV_ALLOW_EAGER_ATTN=0`，将复用当前 flash-attn 服务。

## 回滚方案（如未来需要）
- 重装旧版: `pip3 install flash-attn==2.7.4.post1`
- 临时后备: `INTERNNAV_ALLOW_EAGER_ATTN=1`

## 复现构建（如需重建）
```bash
cd /tmp/flash_attn_src_inspect/flash-attention
export LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib:${LD_LIBRARY_PATH:-}"
export FLASH_ATTN_CUDA_ARCHS="110" MAX_JOBS=4 NVCC_THREADS=4 FLASH_ATTENTION_FORCE_BUILD=TRUE
python3 setup.py bdist_wheel
```
wheel 保留在 `dist/` 目录，可直接重装无需重新编译。

## 时间线
- 12:40 克隆源码
- 12:41 应用补丁
- 12:45 开始编译
- 13:21 编译完成（wheel 生成）
- 13:22 安装并验证架构
- 13:24 推理服务启动成功（flash-attn 模式）
- 13:25 合成推理请求验证通过
