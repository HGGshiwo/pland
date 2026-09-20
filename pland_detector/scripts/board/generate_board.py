#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import re
import shutil
import subprocess
import urllib.request
from io import BytesIO
from pathlib import Path
from PIL import Image


def get_ros_pkg_path(pkg_name):
    """通过 rospack 获取 ROS 功能包路径，支持本地目录回溯兜底"""
    try:
        import rospkg
        return Path(rospkg.RosPack().get_path(pkg_name))
    except Exception:
        pass

    try:
        out = subprocess.check_output(
            ["rospack", "find", pkg_name], text=True, stderr=subprocess.DEVNULL
        ).strip()
        if out and Path(out).exists():
            return Path(out)
    except Exception:
        pass

    # 兜底：基于当前脚本所在源码目录向上查找
    script_file = Path(__file__).resolve()
    for parent in script_file.parents:
        cand = parent / pkg_name
        if cand.exists() and (cand / "package.xml").exists():
            return cand
        cand_sub = parent / "pland" / pkg_name
        if cand_sub.exists() and (cand_sub / "package.xml").exists():
            return cand_sub
    return None


class AprilTagDownloader:
    """负责从官方库下载 Tag（带本地缓存机制）"""
    CACHE_DIR = Path(__file__).resolve().parent / ".tag_cache"

    @classmethod
    def fetch_tag(cls, tag_id, family="tag36h11"):
        cls.CACHE_DIR.mkdir(parents=True, exist_ok=True)
        match = re.match(r"tag(\d+)h(\d+)", family)
        family_prefix = f"tag{match.group(1)}_{int(match.group(2)):02d}_" if match else family.replace("h", "_") + "_"
        filename = f"{family}_{family_prefix}{tag_id:05d}.png"
        cache_path = cls.CACHE_DIR / filename

        if cache_path.exists():
            return Image.open(cache_path).convert("RGB")

        url = f"https://raw.githubusercontent.com/AprilRobotics/apriltag-imgs/master/{family}/{family_prefix}{tag_id:05d}.png"
        req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
        try:
            with urllib.request.urlopen(req) as resp:
                data = resp.read()
            with open(cache_path, "wb") as f:
                f.write(data)
            return Image.open(BytesIO(data)).convert("RGB")
        except Exception as e:
            raise RuntimeError(f"下载失败: {url}\n{e}")


def generate_runway_layout(X_size, start_id=0):
    """OptimizedRunwayLayout: 25标双轨短拉链优化排布"""
    inner_size = X_size * 0.0400
    inner_dist = X_size * 0.0400

    s1, s2, s3, s4 = 0.44 * X_size, 0.42 * X_size, 0.40 * X_size, 0.38 * X_size
    cx1, cx2, cx3, cx4 = 0.28 * X_size, 0.27 * X_size, 0.26 * X_size, 0.25 * X_size

    layout = [
        # 中心十字星团 (ID: start_id + 0~4)
        {"id": start_id + 0, "cx": 0, "cy": 0, "size": inner_size},
        {"id": start_id + 1, "cx": 0, "cy": -inner_dist, "size": inner_size},
        {"id": start_id + 2, "cx": 0, "cy": inner_dist, "size": inner_size},
        {"id": start_id + 3, "cx": -inner_dist, "cy": 0, "size": inner_size},
        {"id": start_id + 4, "cx": inner_dist, "cy": 0, "size": inner_size},
        # 外侧四个大标 (ID: start_id + 5~8)
        {"id": start_id + 5, "cx": -cx1, "cy": -cx1, "size": s1},
        {"id": start_id + 6, "cx": cx2, "cy": -cx2, "size": s2},
        {"id": start_id + 7, "cx": -cx3, "cy": cx3, "size": s3},
        {"id": start_id + 8, "cx": cx4, "cy": cx4, "size": s4},
    ]

    # 无缝延伸跑道刻度 (单臂 5 标，共 20 标)
    runway_size = 0.0400 * X_size
    curr_id = start_id + 9
    for d in [0.08 * X_size, 0.12 * X_size, 0.16 * X_size, 0.20 * X_size, 0.24 * X_size]:
        layout.extend([
            {"id": curr_id + 0, "cx": 0, "cy": -d, "size": runway_size},
            {"id": curr_id + 1, "cx": 0, "cy": d, "size": runway_size},
            {"id": curr_id + 2, "cx": -d, "cy": 0, "size": runway_size},
            {"id": curr_id + 3, "cx": d, "cy": 0, "size": runway_size},
        ])
        curr_id += 4

    return layout


def build_board(board_size_mm=600.0, start_id=0, family="tag36h11", dpi=300, out_prefix="tag_pos_map", auto_copy=False):
    # 动态获取 ROS 路径
    detector_pkg = get_ros_pkg_path("pland_detector")
    sim_pkg = get_ros_pkg_path("pland_sim")

    config_dst = (detector_pkg / "config" / "tag_pos_map.json") if detector_pkg else None
    texture_dst = (sim_pkg / "models" / "apriltag" / "materials" / "textures" / "apriltag.png") if sim_pkg else None

    # 画板像素与物理换算
    pixel_to_meter = 0.0254 / dpi
    board_size_px = int((board_size_mm / 25.4) * dpi)
    canvas_img = Image.new("RGB", (board_size_px, board_size_px), (255, 255, 255))
    center = board_size_px / 2.0

    layout_data = generate_runway_layout(board_size_px * 0.90, start_id)
    ratio = 8.0 / 10.0 if "tag36h11" in family else (6.0 / 8.0 if "tag16h5" in family else 0.75)

    points_3d = {}
    print(f"正在生成画板 ({board_size_mm}mm x {board_size_mm}mm, {board_size_px}px, 共 {len(layout_data)} 个标)...")

    for item in layout_data:
        tid, cx_px, cy_px, size_px = item["id"], item["cx"], item["cy"], int(item["size"])
        img = AprilTagDownloader.fetch_tag(tid, family).resize((size_px, size_px), Image.NEAREST)
        canvas_img.paste(img, (int(center + cx_px - size_px / 2), int(center + cy_px - size_px / 2)))

        cx_m, cy_m = cx_px * pixel_to_meter, cy_px * pixel_to_meter
        half_s = (size_px * ratio / 2.0) * pixel_to_meter
        points_3d[tid] = [
            [-half_s + cx_m, half_s + cy_m, 0.0],
            [half_s + cx_m, half_s + cy_m, 0.0],
            [half_s + cx_m, -half_s + cy_m, 0.0],
            [-half_s + cx_m, -half_s + cy_m, 0.0],
        ]

    # 保存到当前目录
    save_dir = Path.cwd()
    clean_prefix = Path(out_prefix).stem
    json_path = (save_dir / f"{clean_prefix}.json").resolve()
    jpg_path = (save_dir / f"{clean_prefix}.jpg").resolve()

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(points_3d, f, indent=4)
    canvas_img.save(jpg_path, format="JPEG", quality=95)

    print(f"\n✅ 生成成功 (保存于当前目录):")
    print(f"  - 图像: {jpg_path}")
    print(f"  - 配置: {json_path}")

    # 自动同步
    if auto_copy and config_dst and texture_dst:
        config_dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(json_path, config_dst)
        texture_dst.parent.mkdir(parents=True, exist_ok=True)
        canvas_img.save(texture_dst, format="PNG")
        print(f"\n🚀 已自动同步至目标路径:")
        print(f"  -> {config_dst}")
        print(f"  -> {texture_dst}")

    # 提示信息
    print("\n" + "=" * 76)
    print("📢 提示: 请按需更新文件到对应系统目录:")
    print(f"1. 配置文件更新到 pland_detector:\n   cp \"{json_path}\" \"{config_dst}\"")
    print(f"\n2. 图片更新到仿真贴图:\n   cp \"{jpg_path}\" \"{texture_dst}\"")
    print("=" * 76 + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AprilTag 跑道阵列生成工具 (单文件极简版)")
    parser.add_argument("--size-mm", type=float, default=600.0, help="画板实际物理边长(毫米)，默认 600")
    parser.add_argument("--family", default="tag36h11", help="AprilTag 族类 (默认: tag36h11)")
    parser.add_argument("--start-id", type=int, default=0, help="起始 Tag ID (默认: 0)")
    parser.add_argument("--dpi", type=int, default=300, help="图像 DPI (默认: 300)")
    parser.add_argument("--out", default="tag_pos_map", help="输出文件名前缀 (默认: tag_pos_map)")
    parser.add_argument("--copy", action="store_true", help="生成后自动同步至 detector 与仿真模型目录")

    args = parser.parse_args()
    build_board(
        board_size_mm=args.size_mm,
        start_id=args.start_id,
        family=args.family,
        dpi=args.dpi,
        out_prefix=args.out,
        auto_copy=args.copy,
    )