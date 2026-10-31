from pathlib import Path
import argparse, json, os, shutil, signal, fcntl, hashlib, re

ROOT = Path(__file__).resolve().parents[2]
# Runtime data always lives on the identified WD USB disk, never beside code.
STORAGE_MOUNT = Path(
    os.environ.get(
        "APEX_STORAGE_MOUNT",
        (
            "/mnt/apex-data"
            if os.path.ismount("/mnt/apex-data")
            else "/media/descfly/My Passport"
        ),
    )
)
DATA = STORAGE_MOUNT / "apex_n_sharpa" / "data"
EXPECTED_SERIAL = "WD-WX12A367D1A2"
_STORAGE_DEVICE = None


def ensure_storage():
    import subprocess

    global _STORAGE_DEVICE
    mount = STORAGE_MOUNT.resolve()
    if not STORAGE_MOUNT.is_absolute() or not os.path.ismount(mount):
        raise RuntimeError(f"WD硬盘未挂载: {STORAGE_MOUNT}；拒绝回退到系统盘")
    device = subprocess.check_output(
        ["findmnt", "-rn", "-M", str(mount), "-o", "SOURCE"], text=True
    ).strip()
    serials = subprocess.check_output(
        ["lsblk", "-sn", "-o", "SERIAL", device], text=True
    ).split()
    if EXPECTED_SERIAL not in serials:
        raise RuntimeError(f"挂载设备不是指定WD硬盘: {device}, serial={serials}")
    if DATA.resolve() != mount / "apex_n_sharpa" / "data":
        raise RuntimeError("外置数据目录不允许通过符号链接转向其他位置")
    _STORAGE_DEVICE = mount.stat().st_dev
    return mount


def safe(path):
    if _STORAGE_DEVICE is None:
        ensure_storage()
    if (
        not os.path.ismount(STORAGE_MOUNT)
        or STORAGE_MOUNT.stat().st_dev != _STORAGE_DEVICE
    ):
        raise RuntimeError("WD硬盘挂载已改变，停止读写")
    p = Path(path).resolve()
    if DATA.resolve() not in p.parents:
        raise ValueError(f"数据路径必须位于外置硬盘 {DATA}: {p}")
    return p


def sync_directory(path):
    fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def save(path, value):
    path = safe(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("w") as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    tmp.replace(path)
    sync_directory(path.parent)


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def lock(name):
    ensure_storage()
    DATA.mkdir(parents=True, exist_ok=True)
    f = (DATA / ("." + name + ".lock")).open("a")
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return f


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


signal.signal(signal.SIGTERM, interrupted)
import io
import numpy as np


def timestamps(values):
    a = np.asarray(values)
    if a.ndim != 1 or not len(a) or a.dtype.kind not in "iu":
        raise ValueError("时间戳必须为非空一维整数纳秒数组，禁止浮点时间戳")
    if any(int(v) < 0 or int(v) > np.iinfo(np.int64).max for v in a):
        raise ValueError("时间戳超出int64范围")
    a = a.astype("i8")
    if any(int(y) <= int(x) for x, y in zip(a[:-1], a[1:])):
        raise ValueError("时间戳必须唯一且严格递增，不能排序掩盖时钟回退")
    return a


def map_clock(values, source, target, mapping=None):
    a = timestamps(values)
    if not source or not target:
        raise ValueError("必须明确源/目标时钟域")
    if source == target and mapping is None:
        return a
    if not mapping or mapping.get("target_domain") != target:
        raise ValueError(f"缺少{source}到{target}的时钟标定")
    from fractions import Fraction

    scale = Fraction(str(mapping["scale"]))
    if scale <= 0:
        raise ValueError("时钟比例必须为有限正数")
    origin = mapping["source_origin_ns"]
    target_origin = mapping["target_origin_ns"]
    if type(origin) is not int or type(target_origin) is not int:
        raise ValueError("时钟原点必须为整数纳秒")
    # Rational arithmetic avoids float epoch truncation and int64 subtraction overflow.
    converted = [target_origin + round((int(v) - origin) * scale) for v in a]
    return timestamps(converted)


import h5py
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image
from tqdm import tqdm


def align(times, targets, tolerance_ns, causal=False):
    times = timestamps(times)
    targets = timestamps(targets)
    if len(times) == 0 or np.any(np.diff(times) <= 0):
        raise ValueError("源时间戳必须非空、唯一且严格递增")
    if causal:
        idx = np.searchsorted(times, targets, side="right") - 1
        valid = idx >= 0
        idx = np.clip(idx, 0, len(times) - 1)
    else:
        right = np.clip(np.searchsorted(times, targets), 0, len(times) - 1)
        left = np.maximum(right - 1, 0)
        idx = np.where(
            abs(times[left] - targets) <= abs(times[right] - targets), left, right
        )
        valid = np.ones(len(targets), bool)
    error = times[idx] - targets
    valid &= abs(error) <= tolerance_ns
    return idx, valid, error


def rotations(q):
    q = np.asarray(q, dtype="f8")
    norm = np.linalg.norm(q, axis=1)
    good = np.isfinite(q).all(axis=1) & (norm > 1e-8) & (abs(norm - 1) < 0.05)
    q = q / np.where(good, norm, 1)[:, None]
    x, y, z, w = q.T
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z)
    r[:, 0, 1] = 2 * (x * y - z * w)
    r[:, 0, 2] = 2 * (x * z + y * w)
    r[:, 1, 0] = 2 * (x * y + z * w)
    r[:, 1, 1] = 1 - 2 * (x * x + z * z)
    r[:, 1, 2] = 2 * (y * z - x * w)
    r[:, 2, 0] = 2 * (x * z - y * w)
    r[:, 2, 1] = 2 * (y * z + x * w)
    r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return r, good


def convert_pose(values, transform):
    if values.shape[1] != 7:
        raise ValueError("EEF必须为xyz+xyzw七维")
    mat = np.asarray(transform, dtype="f8")
    if (
        mat.shape != (4, 4)
        or not np.isfinite(mat).all()
        or not np.allclose(mat[3], [0, 0, 0, 1])
        or not np.allclose(mat[:3, :3].T @ mat[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(mat[:3, :3]), 1)
    ):
        raise ValueError("标定矩阵必须为合法SE(3)")
    rot, valid = rotations(values[:, 3:])
    rot = mat[:3, :3] @ rot
    xyz = values[:, :3] @ mat[:3, :3].T + mat[:3, 3]
    return np.concatenate([xyz, rot[:, :, 0], rot[:, :, 1]], axis=1), valid


def longest_run(valid):
    padded = np.r_[False, valid, False].astype("i1")
    edges = np.diff(padded)
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    if not len(starts):
        raise ValueError("清洗后无有效连续帧")
    i = np.argmax(ends - starts)
    return slice(starts[i], ends[i])


def stats(array):
    a = np.asarray(array, dtype="f8")
    if a.ndim == 1:
        a = a[:, None]
    return {
        "min": a.min(axis=0).tolist(),
        "max": a.max(axis=0).tolist(),
        "mean": a.mean(axis=0).tolist(),
        "std": a.std(axis=0).tolist(),
        "count": [len(a)],
    }


def read_hands(path, clock_domain, names, capture_id):
    obj = json.loads(safe(path).read_text())
    result = {}
    if (
        not isinstance(capture_id, str)
        or not capture_id.strip()
        or obj.get("capture_id") != capture_id
    ):
        raise ValueError("APEX与Sharpa capture_id缺失或不一致；禁止仅按编号配对")
    # Every stream may have its own clock mapping; no inferred clock offsets.
    for key, stream in obj["streams"].items():
        if key not in {"state/left", "state/right", "action/left", "action/right"}:
            raise ValueError(f"未知Sharpa流: {key}")
        if stream.get("source_role") != key.split("/")[0]:
            raise ValueError(f"{key}: source_role不匹配")
        t = map_clock(
            stream["timestamp_ns"],
            stream["clock_domain"],
            clock_domain,
            stream.get("clock_map"),
        )
        values = np.asarray(stream["values"], dtype="f8")
        if values.shape != (len(t), 22):
            raise ValueError(f"{key}: Sharpa应为N×22")
        order = stream["joint_names"]
        expected = names[32:54] if key.endswith("left") else names[55:77]
        if len(order) != 22 or set(order) != set(expected):
            raise ValueError(f"{key}: Sharpa关节名不匹配")
        values = values[:, [order.index(n) for n in expected]]
        if stream["unit"] == "deg":
            values = np.deg2rad(values)
        elif stream["unit"] != "rad":
            raise ValueError("Sharpa单位必须为rad/deg")
        result[key] = (t, values)
    return result


def check_hdf_storage(group, seen=None):
    seen = set() if seen is None else seen
    address = h5py.h5o.get_info(group.id).addr
    if address in seen:
        return
    seen.add(address)
    for name in group:
        if not isinstance(group.get(name, getlink=True), h5py.HardLink):
            raise ValueError("HDF5不允许外部/软链接")
        obj = group[name]
        if isinstance(obj, h5py.Group):
            check_hdf_storage(obj, seen)
        elif obj.is_virtual or obj.external:
            raise ValueError("HDF5不允许外部存储或虚拟数据集")


def check_group(group, clock, role):
    if group.attrs.get("clock_domain") != clock:
        raise ValueError(f"{group.name}: 时钟域不一致")
    if group.attrs.get("source_role") != role:
        raise ValueError(f"{group.name}: state/action来源标注不符")


def build(hdf5, handfile, dest, config, reference, source_id):
    names = reference["features"]["observation.state"]["names"]
    profile = config.get("profile", "apex_sharpa_78")
    if profile not in {"apex_sharpa_78", "apex_nonhand"}:
        raise ValueError("未知输出profile")
    nonhand = profile == "apex_nonhand"
    dimension = 27 if nonhand else 78
    if nonhand and config.get("allow_missing"):
        raise ValueError("非手部输出禁止缺失维度占位")
    fps = config["fps"]
    tolerance = int(config["max_skew_ms"] * 1e6)
    report = {"source_episode": source_id, "alignment": {}, "missing": []}
    features = {}
    arrays = {}
    images = {}
    aligned_sources = {}
    with h5py.File(safe(hdf5), "r") as h:
        check_hdf_storage(h)
        if not h.attrs.get("complete"):
            raise ValueError("HDF5尚未完成")
        if "time/sample_timestamp_ns" not in h:
            raise ValueError(
                "参考HDF5没有action和统一时钟，须先完成APEX转换，不能将反馈冒充action"
            )
        ts = timestamps(h["time/sample_timestamp_ns"][:])
        if len(ts) < 2 or np.any(np.diff(ts) != round(1e9 / fps)):
            raise ValueError("HDF5采样率/时间轴与配置不一致")
        valid = np.ones(len(ts), bool)
        earliest = ts.copy()
        latest = ts.copy()
        state = np.zeros((len(ts), dimension), dtype="f4")
        action = state.copy()
        mask = np.zeros_like(state, dtype="u1")
        hands = (
            read_hands(
                handfile, h.attrs["clock_domain"], names, h.attrs.get("capture_id", "")
            )
            if handfile and not nonhand
            else {}
        )

        def sample(key, t, values, causal):
            idx, ok, error = align(t, ts, tolerance, causal)
            aligned_sources[key] = {
                "timestamp_ns": np.asarray(t)[idx],
                "source_index": idx,
            }
            report["alignment"][key] = {
                "max_abs_skew_ns": int(abs(error).max()),
                "rejected": int((~ok).sum()),
            }
            valid[:] &= ok
            earliest[:] = np.minimum(earliest, np.asarray(t)[idx])
            latest[:] = np.maximum(latest, np.asarray(t)[idx])
            return np.asarray(values)[idx]

        allowed = set(config.get("allow_missing", []))
        for role, arr in [("observations", state), ("actions", action)]:
            for camera, start in [("left", 0), ("right", 9), ("active_camera", 18)]:
                key = f"{role}/{camera}"
                if key not in h:
                    if key not in allowed:
                        raise ValueError(
                            f"缺少{key}；头部关节不能直接冒充EEF，需FK/标定"
                        )
                    report["missing"].append(key)
                    continue
                g = h[key]
                check_group(
                    g,
                    h.attrs["clock_domain"],
                    "state" if role == "observations" else "action",
                )
                frame = g.attrs["coordinate_frame"]
                if g.attrs.get("unit") != "m":
                    raise ValueError(f"{key}: EEF位置单位必须明确为m")
                transform = (
                    np.eye(4)
                    if frame == "base_link"
                    else config.get("frame_transforms", {}).get(frame)
                )
                if transform is None:
                    raise ValueError(f"缺少{frame}到base_link标定")
                v = sample(key, g["timestamp_ns"][:], g["eef"][:], role == "actions")
                out, ok = convert_pose(v, transform)
                arr[:, start : start + 9] = out
                valid &= ok
            for side, start in ([] if nonhand else [("left", 32), ("right", 55)]):
                key = f"{role}/{side}"
                hk = f'{"state" if role=="observations" else "action"}/{side}'
                if hk not in hands:
                    if hk not in allowed:
                        raise ValueError(f"缺少Sharpa {hk}，不可使用手套值充当真机反馈")
                    report["missing"].append(hk)
                    continue
                t, v = hands[hk]
                arr[:, start : start + 22] = sample(hk, t, v, role == "actions")
        for side, start in [("left", 0), ("right", 9), ("active_camera", 18)]:
            if all(f"{role}/{side}" in h for role in ["observations", "actions"]):
                mask[:, start : start + 9] = 1
        for side, start in ([] if nonhand else [("left", 32), ("right", 55)]):
            if all(f"{role}/{side}" in hands for role in ["state", "action"]):
                mask[:, start : start + 22] = 1
        # A shared validity mask requires both observation and action to exist.
        state[mask == 0] = 0
        action[mask == 0] = 0
        valid &= np.isfinite(state).all(axis=1) & np.isfinite(action).all(axis=1)
        for key, spec in config.get("extra_streams", {}).items():
            if key in {"action", "observation.state", "validity"} or not key.startswith(("observation.", "action.")):
                raise ValueError("额外流输出名称无效")
            g = h[spec["path"]]
            role = "action" if key.startswith("action.") else "state"
            check_group(g, h.attrs["clock_domain"], role)
            is_pose = g.attrs["kind"] == "pose"
            values = sample(spec["path"], g["timestamp_ns"][:],
                            g["eef" if is_pose else "value"][:], role == "action")
            if is_pose:
                frame = g.attrs["coordinate_frame"]
                transform = np.eye(4) if frame == "base_link" else config.get("frame_transforms", {}).get(frame)
                if transform is None or g.attrs["unit"] != "m":
                    raise ValueError("额外位姿缺少坐标变换或单位")
                values, ok = convert_pose(values, transform)
                valid &= ok
            if values.shape != (len(ts), len(spec["names"])):
                raise ValueError("额外流维数与名称不一致")
            arrays[key] = values.astype("f4")
            valid &= np.isfinite(values).all(axis=1)
            features[key] = {"dtype": "float32", "shape": [values.shape[1]], "names": spec["names"]}
            report.setdefault("extra_streams", {})[key] = {
                "source_topic": str(g.attrs["source_topic"]), "source_role": role,
                "unit": str(g.attrs["unit"]), "source_stage": spec["source_stage"],
                "coordinate_frame": "base_link" if is_pose else "native_joint_coordinates"}
        for role, key in [
            ("observations", "observation.chassis"),
            ("actions", "action.chassis"),
        ]:
            path = f"{role}/chassis"
            if path not in h:
                raise ValueError(f"缺少底盘独立数据: {path}")
            g = h[path]
            check_group(
                g,
                h.attrs["clock_domain"],
                "state" if role == "observations" else "action",
            )
            if g.attrs.get("unit") != "m/s,m/s,rad/s":
                raise ValueError("底盘单位必须为m/s,m/s,rad/s")
            v = sample(path, g["timestamp_ns"][:], g["value"][:], role == "actions")
            if v.shape[1] != 3:
                raise ValueError("底盘向量要求vx,vy,wz")
            arrays[key] = v.astype("f4")
            valid &= np.isfinite(arrays[key]).all(axis=1)
            features[key] = {
                "dtype": "float32",
                "shape": [3],
                "names": ["vx_m_s", "vy_m_s", "wz_rad_s"],
            }
        for source, key in [
            ("active_camera", "head_color"),
            ("left", "left_wrist_camera"),
            ("right", "right_wrist_camera"),
        ]:
            group = f"images/{source}"
            if group not in h:
                raise ValueError(f"缺少图像{group}，需显式配置四宫格裁剪区域")
            g = h[group]
            check_group(g, h.attrs["clock_domain"], "image")
            idx, ok, error = align(g["timestamp_ns"][:], ts, tolerance)
            valid &= ok
            output = []
            size = None
            earliest[:] = np.minimum(earliest, g["timestamp_ns"][:][idx])
            latest[:] = np.maximum(latest, g["timestamp_ns"][:][idx])
            for j in idx:
                data = g["jpeg"][int(j)].tobytes()
                try:
                    with Image.open(io.BytesIO(data)) as im:
                        im.load()
                        if im.mode != "RGB" or im.format != "JPEG":
                            raise ValueError("图像必须为RGB JPEG")
                        current = (im.height, im.width)
                    if size is not None and size != current:
                        raise ValueError("图像尺寸改变")
                    size = current
                    output.append(data)
                except (OSError, ValueError):
                    output.append(None)
            valid &= np.array([x is not None for x in output])
            name = "observation.images." + key
            images[name] = output
            aligned_sources[name] = {
                "timestamp_ns": g["timestamp_ns"][:][idx],
                "source_index": idx,
            }
            if size is None:
                raise ValueError("相机所有图像均损坏")
            features[name] = {
                "dtype": "image",
                "shape": [3, *size],
                "names": ["channels", "height", "width"],
            }
            report["alignment"][name] = {
                "max_abs_skew_ns": int(abs(error).max()),
                "rejected": int((~ok).sum()),
            }
    spread = latest - earliest
    valid &= spread <= tolerance
    report["cross_stream_max_spread_ns"] = int(spread.max())
    chosen = longest_run(valid)
    report["retained_max_spread_ns"] = int(spread[chosen].max())
    n = int(chosen.stop - chosen.start)
    if n < config.get("min_frames", 2):
        raise ValueError("有效连续帧数不足")
    report.update(
        input_frames=len(ts),
        output_frames=n,
        dropped_frames=len(ts) - n,
        original_start_ns=int(ts[chosen.start]),
        original_end_ns=int(ts[chosen.stop - 1]),
        cleaning="longest_contiguous_valid_run; no time compression",
    )
    arrays.update({"observation.state": state, "action": action, "validity": mask})
    for key in ["observation.state", "action", "validity"]:
        features[key] = dict(reference["features"][key])
        if nonhand:
            features[key].update(shape=[27], names=names[:27])
    arrays = {k: v[chosen] for k, v in arrays.items()}
    arrays.update(
        timestamp=np.arange(n, dtype="f4") / fps,
        frame_index=np.arange(n, dtype="i8"),
        episode_index=np.zeros(n, dtype="i8"),
        index=np.arange(n, dtype="i8"),
        task_index=np.zeros(n, dtype="i8"),
    )
    columns = {}
    for key, v in arrays.items():
        dtype = pa.from_numpy_dtype(v.dtype)
        columns[key] = (
            pa.array(v.tolist(), type=pa.list_(dtype, v.shape[1]))
            if v.ndim == 2
            else pa.array(v, type=dtype)
        )
        if key not in features:
            features[key] = {"dtype": str(v.dtype), "shape": [1], "names": None}
    image_type = pa.struct([("bytes", pa.binary()), ("path", pa.string())])
    for key, rows in images.items():
        columns[key] = pa.array(
            [{"bytes": v, "path": None} for v in rows[chosen]], type=image_type
        )
    folder = safe(dest / "data/chunk-000")
    folder.mkdir(parents=True)
    hf = {}
    for key, v in arrays.items():
        value = {"_type": "Value", "dtype": str(v.dtype)}
        hf[key] = (
            {"_type": "Sequence", "length": v.shape[1], "feature": value}
            if v.ndim == 2
            else value
        )
    for key in images:
        hf[key] = {"_type": "Image"}
    table = pa.table(columns).replace_schema_metadata(
        {"huggingface": json.dumps({"info": {"features": hf}})}
    )
    pq.write_table(table, folder / "episode_000000.parquet", compression="zstd")
    meta = dest / "meta"
    meta.mkdir()
    info = {
        "codebase_version": "v2.1",
        "robot_type": "apex_nonhand" if nonhand else "apex_sharpa",
        "fps": fps,
        "total_episodes": 1,
        "total_frames": n,
        "total_tasks": 1,
        "total_videos": 0,
        "total_chunks": 1,
        "chunks_size": 1000,
        "splits": {"train": "0:1"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": None,
        "features": features,
        "action_mask": mask[0].astype(bool).tolist(),
        "physical_action_dim": int(mask[0].sum()),
        "valid_action_dim": int(mask[0].sum()),
        "qpos_encoding": "apex_27d_base_link_xyz_rot6d_columns_v1" if nonhand else "egoav_78d_absolute_public_frame_xyz_rot6d_columns_v1",
        "coordinate_frame": "base_link",
        "profile": profile,
        "egoav78_wire_compatible": False,
        "head_ee_definition": config.get("head_ee_definition"),
    }
    save(meta / "info.json", info)
    save(meta / "cleaning_report.json", report)
    statistics = {k: stats(v) for k, v in arrays.items()}
    for key, rows in images.items():
        pixels = (
            np.asarray(Image.open(io.BytesIO(v)), dtype="f8") / 255
            for v in rows[chosen]
        )
        sums = np.zeros(3)
        squares = np.zeros(3)
        lo = np.ones(3)
        hi = np.zeros(3)
        count = 0
        for pix in pixels:
            flat = pix.reshape(-1, 3)
            sums += flat.sum(0)
            squares += (flat * flat).sum(0)
            count += len(flat)
            lo = np.minimum(lo, flat.min(0))
            hi = np.maximum(hi, flat.max(0))
        mean = sums / count
        std = np.sqrt(np.maximum(squares / count - mean * mean, 0))
        statistics[key] = {
            k: v[:, None, None].tolist()
            for k, v in [("min", lo), ("max", hi), ("mean", mean), ("std", std)]
        }
        statistics[key]["count"] = [n]
    save(meta / "stats.json", statistics)
    for name, obj in [
        ("tasks", {"task_index": 0, "task": config["task_text"]}),
        ("episodes", {"episode_index": 0, "tasks": [config["task_text"]], "length": n}),
        ("episodes_stats", {"episode_index": 0, "stats": statistics}),
    ]:
        (meta / (name + ".jsonl")).write_text(
            json.dumps(obj, ensure_ascii=False, allow_nan=False) + "\n"
        )
    # Keep original nanosecond timeline and exact source episode outside training schema.
    save(
        meta / "source_timestamps.json",
        {
            "timestamp_ns": ts[chosen].tolist(),
            "source_episode": source_id,
            "aligned_sources": {
                k: {field: values[chosen].tolist() for field, values in entry.items()}
                for k, entry in aligned_sources.items()
            },
        },
    )


def main():
    p = argparse.ArgumentParser(
        description="HDF5+Sharpa对齐、清洗、事务式导出LeRobot v2.1"
    )
    p.add_argument("--task", required=True)
    p.add_argument("--config", default=str(DATA / "config/lerobot.json"))
    a = p.parse_args()
    if not re.fullmatch(r"[\w-]+", a.task):
        p.error("task仅允许字母数字下划线及连字符")
    guard = lock("hdf52lerobot")
    config_bytes = safe(a.config).read_bytes()
    config = json.loads(config_bytes)
    config_digest = hashlib.sha256(config_bytes).hexdigest()
    if (
        not np.isfinite(config["fps"])
        or not 0 < config["fps"] <= 1e9
        or not np.isfinite(config["max_skew_ms"])
        or config["max_skew_ms"] < 0
    ):
        raise ValueError("fps/max_skew_ms不合法")
    config.setdefault("task_text", a.task)
    reference_bytes = safe(DATA / "references/info.json").read_bytes()
    reference = json.loads(reference_bytes)
    reference_digest = hashlib.sha256(reference_bytes).hexdigest()
    source = safe(DATA / "hdf5" / a.task)
    dest = safe(DATA / "lerobot" / a.task)
    dest.mkdir(parents=True, exist_ok=True)
    for path in dest.glob(".*.partial"):
        shutil.rmtree(path)
    files = sorted(source.glob("hdf5_[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9].hdf5"))
    if not files:
        raise ValueError("没有HDF5文件")
    for file in tqdm(files, desc="hdf52lerobot + clean", unit="episode"):
        eid = file.stem.split("_")[1]
        out = dest / f"episode_{eid}"
        tmp = dest / f".episode_{eid}.partial"
        checkpoint = dest / "progress.json"
        save(checkpoint, dict(episode=eid, status="running"))
        try:
            if (
                digest(safe(a.config)) != config_digest
                or digest(safe(DATA / "references/info.json")) != reference_digest
            ):
                raise ValueError("运行期间配置或参考规范改变，请重新启动命令")
            hand = safe(
                DATA
                / "raw_data/sharpa_data"
                / a.task
                / f"episode_{eid}/payload/sharpa.json"
            )
            if not hand.exists():
                hand = None
            provenance = {
                "hdf5": digest(safe(file)),
                "sharpa": digest(hand) if hand else None,
                "config": config_digest,
                "reference": reference_digest,
                "converter_revision": 2,
            }
            if out.exists():
                if (
                    json.loads(safe(out / "meta/provenance.json").read_text())
                    != provenance
                ):
                    raise ValueError(f"输入已改变: {out}")
                record = json.loads(safe(out / "meta/completion.json").read_text())
                actual = {
                    str(p.relative_to(out))
                    for p in out.rglob("*")
                    if p.is_file() and p != out / "meta/completion.json"
                }
                if actual != set(record):
                    raise ValueError("已生成LeRobot文件缺失或增加")
                for rel, checksum in record.items():
                    path = safe(out / rel)
                    if out not in path.parents or digest(path) != checksum:
                        raise ValueError(f"已生成LeRobot文件损坏: {path}")
                save(checkpoint, dict(episode=eid, status="complete"))
                continue
            build(file, hand, tmp, config, reference, eid)
            if (
                digest(safe(file)) != provenance["hdf5"]
                or (hand and digest(hand) != provenance["sharpa"])
                or digest(safe(a.config)) != provenance["config"]
                or digest(safe(DATA / "references/info.json")) != reference_digest
            ):
                raise ValueError("转换期间输入或配置改变")
            save(tmp / "meta/provenance.json", provenance)
            record = {
                str(p.relative_to(tmp)): digest(p)
                for p in tmp.rglob("*")
                if p.is_file()
            }
            save(tmp / "meta/completion.json", record)
            for pth in tmp.rglob("*"):
                if pth.is_file():
                    with pth.open("rb") as f:
                        os.fsync(f.fileno())
            tmp.rename(out)
            sync_directory(dest)
            save(checkpoint, dict(episode=eid, status="complete"))
            tqdm.write(f"完成 HDF5+Sharpa {eid}")
        except BaseException as exc:
            shutil.rmtree(tmp, ignore_errors=True)
            save(checkpoint, dict(episode=eid, status="interrupted", error=str(exc)))
            raise


if __name__ == "__main__":
    main()
