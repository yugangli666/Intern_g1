import copy
import itertools
import os
import re
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import torch

sys.path.append(str(Path(__file__).parent.parent.parent))

from collections import OrderedDict

from PIL import Image, ImageDraw, ImageFont
from transformers import AutoProcessor

from internnav.model.basemodel.internvla_n1.internvla_n1 import InternVLAN1ForCausalLM
from internnav.model.utils.vln_utils import S2Output, split_and_clean, traj_to_actions

DEFAULT_IMAGE_TOKEN = "<image>"


def _load_visual_font(size, bold=False):
    font_paths = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc" if bold else "",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for font_path in font_paths:
        if font_path and os.path.exists(font_path):
            return ImageFont.truetype(font_path, size)
    return ImageFont.load_default()


def _text_size(draw, text, font):
    bbox = draw.textbbox((0, 0), text, font=font)
    return bbox[2] - bbox[0], bbox[3] - bbox[1]


def _wrap_text(draw, text, font, max_width):
    lines = []
    current = ""
    for word in text.split(" "):
        candidate = word if not current else current + " " + word
        if _text_size(draw, candidate, font)[0] <= max_width:
            current = candidate
        else:
            if current:
                lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def _to_pixel(point):
    if point is None or len(point) < 2:
        return None
    return [int(round(float(point[0]))), int(round(float(point[1])))]


def _point_in_bounds(point, width, height):
    return point is not None and 0 <= point[0] < width and 0 <= point[1] < height


def _choose_visual_point(pixel_goal, raw_coord, width, height):
    raw_point = _to_pixel(raw_coord)
    swapped_point = _to_pixel(pixel_goal)
    if _point_in_bounds(raw_point, width, height):
        return raw_point
    if _point_in_bounds(swapped_point, width, height):
        return swapped_point

    point = raw_point or swapped_point or [width // 2, height // 2]
    return [min(max(point[0], 0), width - 1), min(max(point[1], 0), height - 1)]


def save_pixel_goal_visualization(image, pixel_goal, output_path, raw_coord=None):
    base_image = image.convert("RGB")
    width, height = base_image.size
    x, y = _choose_visual_point(pixel_goal, raw_coord, width, height)

    draw = ImageDraw.Draw(base_image)
    red = (238, 18, 24)
    line_width = max(3, width // 180)
    radius = max(8, width // 55)
    gap = max(4, radius // 2)
    arm = max(18, radius * 2)

    draw.line((x - arm, y, x - gap, y), fill=red, width=line_width)
    draw.line((x + gap, y, x + arm, y), fill=red, width=line_width)
    draw.line((x, y - arm, x, y - gap), fill=red, width=line_width)
    draw.line((x, y + gap, x, y + arm), fill=red, width=line_width)
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=red)
    inner_radius = max(4, radius - line_width - 2)
    draw.ellipse((x - inner_radius, y - inner_radius, x + inner_radius, y + inner_radius), fill=(255, 255, 255))
    center_radius = max(2, radius // 4)
    draw.ellipse((x - center_radius, y - center_radius, x + center_radius, y + center_radius), fill=red)

    footer_height = max(62, height // 8)
    canvas = Image.new("RGB", (width, height + footer_height), (255, 255, 255))
    canvas.paste(base_image, (0, 0))
    canvas_draw = ImageDraw.Draw(canvas)
    canvas_draw.rectangle((0, 0, width - 1, height + footer_height - 1), outline=(220, 220, 220), width=2)

    label = "模型输出:"
    coord_text = f"({x},{y})"
    label_font = _load_visual_font(max(18, min(30, width // 22)), bold=True)
    coord_font = _load_visual_font(max(18, min(30, width // 22)), bold=True)
    label_w, label_h = _text_size(canvas_draw, label, label_font)
    coord_w, coord_h = _text_size(canvas_draw, coord_text, coord_font)

    margin = max(12, width // 40)
    text_y = height + (footer_height - max(label_h, coord_h)) // 2
    canvas_draw.text((margin, text_y), label, fill=(30, 30, 30), font=label_font)

    pill_pad_x = max(14, width // 45)
    pill_pad_y = max(8, footer_height // 7)
    pill_w = coord_w + pill_pad_x * 2
    pill_h = coord_h + pill_pad_y * 2
    pill_x = min(width - pill_w - margin, margin + label_w + max(16, width // 30))
    pill_y = height + (footer_height - pill_h) // 2
    canvas_draw.rounded_rectangle((pill_x, pill_y, pill_x + pill_w, pill_y + pill_h), radius=10, fill=red)
    canvas_draw.text((pill_x + pill_pad_x, pill_y + pill_pad_y - 1), coord_text, fill=(255, 255, 255), font=coord_font)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    canvas.save(output_path, quality=95)


def _read_llm_output(record_dir, step):
    llm_path = Path(record_dir) / f"llm_output_ {step:03d}.txt"
    if not llm_path.exists():
        return ""
    return llm_path.read_text(errors="ignore").strip()


def _file_time(path):
    return datetime.fromtimestamp(Path(path).stat().st_mtime).strftime("%H:%M:%S")


def save_experiment_record_pages(record_dir, run_id, instruction):
    record_dir = Path(record_dir)
    vis_files = sorted(record_dir.glob("pixel_goal_vis_*.jpg"))
    if not vis_files:
        return []

    page_w = 1500
    margin = 42
    header_h = 220
    cell_gap = 34
    cols = 2
    cell_w = (page_w - 2 * margin - cell_gap) // 2
    image_h = 410
    title_h = 58
    cell_h = title_h + image_h
    page_h = header_h + margin + 2 * cell_h + cell_gap + margin

    title_font = _load_visual_font(34, bold=True)
    sub_font = _load_visual_font(22)
    meta_font = _load_visual_font(18)
    card_title_font = _load_visual_font(22, bold=True)
    card_meta_font = _load_visual_font(18)

    page_paths = []
    for page_idx, start in enumerate(range(0, len(vis_files), 4), 1):
        files = vis_files[start : start + 4]
        total_pages = (len(vis_files) + 3) // 4
        canvas = Image.new("RGB", (page_w, page_h), (247, 248, 250))
        draw = ImageDraw.Draw(canvas)

        draw.rectangle((0, 0, page_w, header_h), fill=(255, 255, 255))
        draw.text(
            (margin, 28),
            f"Experiment Record - {run_id}  ({page_idx}/{total_pages})",
            fill=(25, 30, 38),
            font=title_font,
        )
        y = 82
        draw.text((margin, y), "导航指令 / Instruction:", fill=(60, 65, 75), font=sub_font)
        y += 32
        for line in _wrap_text(draw, instruction or "N/A", sub_font, page_w - 2 * margin):
            draw.text((margin, y), line, fill=(20, 20, 20), font=sub_font)
            y += 28
        draw.text((margin, header_h - 28), f"Source: {record_dir}", fill=(105, 110, 120), font=meta_font)

        for i, img_path in enumerate(files):
            match = re.search(r"pixel_goal_vis_(\d+)", img_path.name)
            step = int(match.group(1)) if match else -1
            row = i // cols
            col = i % cols
            x0 = margin + col * (cell_w + cell_gap)
            y0 = header_h + margin + row * (cell_h + cell_gap)
            x1 = x0 + cell_w
            y1 = y0 + cell_h

            draw.rounded_rectangle(
                (x0, y0, x1, y1),
                radius=12,
                fill=(255, 255, 255),
                outline=(218, 222, 228),
                width=2,
            )
            llm_output = _read_llm_output(record_dir, step)
            card_title = f"Step {step:04d}" + ("  look_down" if "look_down" in img_path.name else "")
            card_meta = f"LLM output: {llm_output or 'N/A'}    Time: {_file_time(img_path)}"
            draw.text((x0 + 18, y0 + 12), card_title, fill=(24, 31, 42), font=card_title_font)
            draw.text((x0 + 18, y0 + 38), card_meta, fill=(90, 95, 105), font=card_meta_font)

            image = Image.open(img_path).convert("RGB")
            image.thumbnail((cell_w - 24, image_h - 16), Image.Resampling.LANCZOS)
            ix = x0 + (cell_w - image.width) // 2
            iy = y0 + title_h + (image_h - image.height) // 2
            canvas.paste(image, (ix, iy))

        out_path = record_dir / f"experiment_record_{page_idx:02d}.jpg"
        canvas.save(out_path, quality=95)
        page_paths.append(out_path)

    return page_paths


class InternVLAN1AsyncAgent:
    def __init__(self, args):
        self.device = torch.device(args.device)
        self._start_new_run()
        print(f"args.model_path{args.model_path}")

        # 强制要求 flash-attn 可用：缺失或无法加载 CUDA 扩展时立即报错，不静默回退 eager。
        try:
            import flash_attn  # noqa: F401
            import flash_attn_2_cuda  # noqa: F401
        except Exception as exc:  # ImportError / OSError(undefined symbol) 等
            raise RuntimeError(
                "flash_attention_2 is required but flash-attn could not be imported: "
                f"{type(exc).__name__}: {exc}. "
                "确认已安装 flash-attn 并将其目录加入 PYTHONPATH "
                "(export PYTHONPATH=/usr/local/lib/python3.12/dist-packages/flash_attn:$PYTHONPATH)。"
            ) from exc
        print(f"[INFO] flash-attn {flash_attn.__version__} 已启用 (attn_implementation=flash_attention_2)")

        self.model = InternVLAN1ForCausalLM.from_pretrained(
            args.model_path,
            torch_dtype=torch.bfloat16,
            attn_implementation="flash_attention_2",  # 强制启用 flash-attn，失败直接报错不回退 eager
            device_map={"": self.device},
        )
        self.model.eval()
        self.model.to(self.device)

        self.processor = AutoProcessor.from_pretrained(args.model_path)
        self.processor.tokenizer.padding_side = 'left'

        self.resize_w = args.resize_w
        self.resize_h = args.resize_h
        self.num_history = args.num_history
        self.PLAN_STEP_GAP = args.plan_step_gap

        prompt = "You are an autonomous navigation assistant. Your task is to <instruction>. Where should you go next to stay on track? Please output the next waypoint's coordinates in the image. Please output STOP when you have successfully completed the task."
        answer = ""
        self.conversation = [{"from": "human", "value": prompt}, {"from": "gpt", "value": answer}]
        self.conjunctions = [
            'you can see ',
            'in front of you is ',
            'there is ',
            'you can spot ',
            'you are toward the ',
            'ahead of you is ',
            'in your sight is ',
        ]

        self.actions2idx = OrderedDict(
            {
                'STOP': [0],
                "↑": [1],
                "←": [2],
                "→": [3],
                "↓": [5],
            }
        )

        self.rgb_list = []
        self.depth_list = []
        self.pose_list = []
        self.episode_idx = 0
        self.conversation_history = []
        self.llm_output = ""
        self.past_key_values = None
        self.last_s2_idx = -100

        # output
        self.output_action = None
        self.output_latent = None
        self.output_pixel = None
        self.pixel_goal_rgb = None
        self.pixel_goal_depth = None

    def _start_new_run(self):
        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_dir = f"test_data/{self.run_id}"
        self.record_dir = f"experiment_records/{self.run_id}"
        self.current_instruction = ""
        os.makedirs(self.save_dir, exist_ok=True)
        os.makedirs(self.record_dir, exist_ok=True)

    def _save_run_instruction(self, instruction):
        instruction = str(instruction or "").strip()
        if not instruction:
            return
        self.current_instruction = instruction
        for directory in (self.save_dir, self.record_dir):
            with open(f"{directory}/experiment_instruction.txt", "w") as f:
                f.write(instruction + "\n")

    def _save_llm_output(self):
        filename = f"llm_output_{self.episode_idx: 04d}.txt"
        for directory in (self.save_dir, self.record_dir):
            with open(f"{directory}/{filename}", "w") as f:
                f.write(self.llm_output)

    def _save_pixel_goal_visualization(self, image, pixel_goal, raw_coord, suffix):
        filename = f"pixel_goal_vis_{self.episode_idx:04d}{suffix}.jpg"
        test_data_path = f"{self.save_dir}/{filename}"
        record_path = f"{self.record_dir}/{filename}"
        save_pixel_goal_visualization(image, pixel_goal, test_data_path, raw_coord=raw_coord)
        shutil.copy2(test_data_path, record_path)
        save_experiment_record_pages(self.record_dir, self.run_id, self.current_instruction)

    def reset(self):
        self.rgb_list = []
        self.depth_list = []
        self.pose_list = []
        self.episode_idx = 0
        self.conversation_history = []
        self.llm_output = ""
        self.past_key_values = None

        self.output_action = None
        self.output_latent = None
        self.output_pixel = None
        self.pixel_goal_rgb = None
        self.pixel_goal_depth = None

        self._start_new_run()

    def parse_actions(self, output):
        action_patterns = '|'.join(re.escape(action) for action in self.actions2idx)
        regex = re.compile(action_patterns)
        matches = regex.findall(output)
        actions = [self.actions2idx[match] for match in matches]
        actions = itertools.chain.from_iterable(actions)
        return list(actions)

    def step_no_infer(self, rgb, depth, pose):
        image = Image.fromarray(rgb).convert('RGB')
        image = image.resize((self.resize_w, self.resize_h))
        self.rgb_list.append(image)
        image.save(f"{self.save_dir}/debug_raw_{self.episode_idx: 04d}.jpg")
        self.episode_idx += 1

    def trajectory_tovw(self, trajectory, kp=1.0):
        subgoal = trajectory[-1]
        linear_vel, angular_vel = kp * np.linalg.norm(subgoal[:2]), kp * subgoal[2]
        linear_vel = np.clip(linear_vel, 0, 0.5)
        angular_vel = np.clip(angular_vel, -0.5, 0.5)
        return linear_vel, angular_vel

    def step(self, rgb, depth, pose, instruction, intrinsic, look_down=False):
        dual_sys_output = S2Output()
        no_output_flag = self.output_action is None and self.output_latent is None
        if (self.episode_idx - self.last_s2_idx > self.PLAN_STEP_GAP) or look_down or no_output_flag:
            self.output_action, self.output_latent, self.output_pixel = self.step_s2(
                rgb, depth, pose, instruction, intrinsic, look_down
            )
            self.last_s2_idx = self.episode_idx
            dual_sys_output.output_pixel = self.output_pixel
            self.pixel_goal_rgb = copy.deepcopy(rgb)
            self.pixel_goal_depth = copy.deepcopy(depth)
        else:
            self.step_no_infer(rgb, depth, pose)

        if self.output_action is not None:
            dual_sys_output.output_action = copy.deepcopy(self.output_action)
            self.output_action = None
        elif self.output_latent is not None:
            processed_pixel_rgb = np.array(Image.fromarray(self.pixel_goal_rgb).resize((224, 224))) / 255
            processed_pixel_depth = np.array(Image.fromarray(self.pixel_goal_depth).resize((224, 224)))
            processed_rgb = np.array(Image.fromarray(rgb).resize((224, 224))) / 255
            processed_depth = np.array(Image.fromarray(depth).resize((224, 224)))
            rgbs = (
                torch.stack([torch.from_numpy(processed_pixel_rgb), torch.from_numpy(processed_rgb)])
                .unsqueeze(0)
                .to(self.device)
            )
            depths = (
                torch.stack([torch.from_numpy(processed_pixel_depth), torch.from_numpy(processed_depth)])
                .unsqueeze(0)
                .unsqueeze(-1)
                .to(self.device)
            )
            trajectories = self.step_s1(self.output_latent, rgbs, depths)

            dual_sys_output.output_trajectory = traj_to_actions(trajectories, use_discrate_action=False)

        return dual_sys_output

    def step_s2(self, rgb, depth, pose, instruction, intrinsic, look_down=False):
        self._save_run_instruction(instruction)
        raw_image = Image.fromarray(rgb).convert('RGB')
        image = raw_image.copy()
        if not look_down:
            image = image.resize((self.resize_w, self.resize_h))
            self.rgb_list.append(image)
            image.save(f"{self.save_dir}/debug_raw_{self.episode_idx: 04d}.jpg")
        else:
            image.save(f"{self.save_dir}/debug_raw_{self.episode_idx: 04d}_look_down.jpg")
        if not look_down:
            self.conversation_history = []
            self.past_key_values = None

            sources = copy.deepcopy(self.conversation)
            sources[0]["value"] = sources[0]["value"].replace('<instruction>.', instruction)
            cur_images = self.rgb_list[-1:]
            if self.episode_idx == 0:
                history_id = []
            else:
                history_id = np.unique(np.linspace(0, self.episode_idx - 1, self.num_history, dtype=np.int32)).tolist()
                placeholder = (DEFAULT_IMAGE_TOKEN + '\n') * len(history_id)
                sources[0]["value"] += f' These are your historical observations: {placeholder}.'

            history_id = sorted(history_id)
            self.input_images = [self.rgb_list[i] for i in history_id] + cur_images
            input_img_id = 0
            self.episode_idx += 1
        else:
            self.input_images.append(image)
            input_img_id = -1
            assert self.llm_output != "", "Last llm_output should not be empty when look down"
            sources = [{"from": "human", "value": ""}, {"from": "gpt", "value": ""}]
            self.conversation_history.append(
                {'role': 'assistant', 'content': [{'type': 'text', 'text': self.llm_output}]}
            )

        prompt = self.conjunctions[0] + DEFAULT_IMAGE_TOKEN
        sources[0]["value"] += f" {prompt}."
        prompt_instruction = copy.deepcopy(sources[0]["value"])
        parts = split_and_clean(prompt_instruction)

        content = []
        for i in range(len(parts)):
            if parts[i] == "<image>":
                content.append({"type": "image", "image": self.input_images[input_img_id]})
                input_img_id += 1
            else:
                content.append({"type": "text", "text": parts[i]})

        self.conversation_history.append({'role': 'user', 'content': content})

        text = self.processor.apply_chat_template(self.conversation_history, tokenize=False, add_generation_prompt=True)

        inputs = self.processor(text=[text], images=self.input_images, return_tensors="pt").to(self.device)
        t0 = time.time()
        with torch.no_grad():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=128,
                do_sample=False,
                # use_cache=True,
                # past_key_values=self.past_key_values,
                return_dict_in_generate=True,
                # raw_input_ids=copy.deepcopy(inputs.input_ids),
            )
        output_ids = outputs.sequences

        t1 = time.time()
        self.llm_output = self.processor.tokenizer.decode(
            output_ids[0][inputs.input_ids.shape[1] :], skip_special_tokens=True
        )
        self._save_llm_output()
        self.last_output_ids = copy.deepcopy(output_ids[0])
        self.past_key_values = copy.deepcopy(outputs.past_key_values)
        print(f"output {self.episode_idx}  {self.llm_output} cost: {t1 - t0}s")
        if bool(re.search(r'\d', self.llm_output)):
            coord = [int(c) for c in re.findall(r'\d+', self.llm_output)]
            pixel_goal = [int(coord[1]), int(coord[0])]
            suffix = "_look_down" if look_down else ""
            self._save_pixel_goal_visualization(raw_image, pixel_goal, coord[:2], suffix)
            image_grid_thw = torch.cat([thw.unsqueeze(0) for thw in inputs.image_grid_thw], dim=0)
            pixel_values = inputs.pixel_values
            t0 = time.time()
            with torch.no_grad():
                traj_latents = self.model.generate_latents(output_ids, pixel_values, image_grid_thw)
                return None, traj_latents, pixel_goal

        else:
            action_seq = self.parse_actions(self.llm_output)
            return action_seq, None, None

    def step_s1(self, latent, rgb, depth):
        with torch.no_grad():
            all_trajs = self.model.generate_traj(latent, rgb, depth)
        return all_trajs.detach()
