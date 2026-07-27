#!/usr/bin/env python3
"""
简单推理测试脚本 - 测试http_internvla_server
"""
import requests
import numpy as np
from PIL import Image
import io
import json

def create_test_image(width=640, height=480):
    """创建测试RGB图像"""
    img = np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
    return Image.fromarray(img)

def create_test_depth(width=640, height=480):
    """创建测试深度图 (32-bit float)"""
    depth = np.random.rand(height, width).astype(np.float32) * 5.0  # 0-5米深度
    # 转换为16位整数 (乘以10000表示毫米)
    depth_uint16 = (depth * 10000).astype(np.uint16)
    return Image.fromarray(depth_uint16, mode='I;16')

def test_inference_server(url='http://127.0.0.1:5801/eval_dual', instruction="go forward"):
    """测试推理服务器"""

    # 创建测试数据
    rgb_img = create_test_image()
    depth_img = create_test_depth()

    # 转换为字节流
    rgb_bytes = io.BytesIO()
    rgb_img.save(rgb_bytes, format='JPEG')
    rgb_bytes.seek(0)

    depth_bytes = io.BytesIO()
    depth_img.save(depth_bytes, format='PNG')
    depth_bytes.seek(0)

    # 准备请求数据
    data = {"reset": True, "instruction": instruction}
    json_data = json.dumps(data)

    files = {
        'image': ('rgb_image.jpg', rgb_bytes, 'image/jpeg'),
        'depth': ('depth_image.png', depth_bytes, 'image/png'),
    }

    print(f"发送推理请求到: {url}")
    print(f"指令: {instruction}")

    try:
        print("正在发送请求...")
        response = requests.post(url, files=files, data={'json': json_data}, timeout=60)
        print(f"\n✓ 响应状态码: {response.status_code}")

        if response.status_code == 200:
            result = response.json()
            print(f"\n✓ 推理成功!")
            print(f"\n解析结果:")
            if 'trajectory' in result:
                traj = result['trajectory']
                print(f"  - 轨迹点数量: {len(traj)}")
                if len(traj) > 0:
                    print(f"  - 轨迹预览 (前3个点):")
                    for i, point in enumerate(traj[:3]):
                        print(f"    点{i+1}: {point}")
            if 'discrete_action' in result:
                print(f"  - 离散动作: {result['discrete_action']}")
            if 'pixel_goal' in result:
                print(f"  - 像素目标: {result['pixel_goal']}")

            print(f"\n✓ 推理测试成功！")
            return result
        else:
            print(f"✗ 服务器返回错误: {response.text}")
            return None

    except requests.exceptions.ConnectionError:
        print("✗ 错误: 无法连接到推理服务器")
        return None
    except requests.exceptions.Timeout:
        print("✗ 错误: 请求超时")
        return None
    except Exception as e:
        print(f"✗ 错误: {e}")
        return None

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='测试InternVLA推理服务器')
    parser.add_argument('--url', type=str, default='http://127.0.0.1:5801/eval_dual',
                        help='推理服务器URL')
    parser.add_argument('--instruction', type=str, default='go forward',
                        help='导航指令')
    args = parser.parse_args()

    test_inference_server(args.url, args.instruction)
