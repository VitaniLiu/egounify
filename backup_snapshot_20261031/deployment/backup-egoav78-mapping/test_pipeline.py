"""Synthetic tests only. Never uses robot or real recordings."""

import importlib.util
import json, subprocess, sys, io, shutil, uuid, os, errno
from pathlib import Path
import numpy as np
import pytest
from PIL import Image
import pyarrow.parquet as pq

ROOT = Path(__file__).resolve().parents[1]
DATA = (
    Path(
        os.environ.get(
            "APEX_STORAGE_MOUNT",
            (
                "/mnt/apex-data"
                if os.path.ismount("/mnt/apex-data")
                else "/media/descfly/My Passport"
            ),
        )
    )
    / "apex_n_sharpa/data"
)


def module(step):
    spec = importlib.util.spec_from_file_location(step, ROOT / "src" / step / "main.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


lr = module("hdf52lerobot")
lr.ensure_storage()


def test_measured_head_fk_and_pinned_model():
    import hashlib
    from types import SimpleNamespace

    m = module("apex2hdf5")
    path = DATA / "test_sources" / ("fk_" + uuid.uuid4().hex + ".xml")
    blob = b'''<mujoco><compiler angle="radian"/><worldbody>
    <body name="base"><body name="neck" pos="1 0 0">
    <joint name="yaw" axis="0 0 1"/>
    <body name="eyes_link" pos="1 0 0"/></body></body>
    </worldbody></mujoco>'''
    path.write_bytes(blob)
    spec = dict(model=str(path), sha256=hashlib.sha256(blob).hexdigest(),
                root="base", target="eyes_link")
    try:
        fk = m.ModelPose(spec)
        pose = fk(SimpleNamespace(name=["yaw"], position=[np.pi / 2]))
        assert np.allclose(pose[:3], [1, 1, 0])
        rotations, valid = lr.rotations(pose[None, 3:])
        assert valid.all()
        assert np.allclose(rotations[0], [[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        pose180 = fk(SimpleNamespace(name=["yaw"], position=[np.pi]))
        assert np.allclose(pose180[:3], [0, 0, 0], atol=1e-12)
        with pytest.raises(ValueError, match="缺少实测关节"):
            fk(SimpleNamespace(name=[], position=[]))
        with pytest.raises(ValueError, match="非有限"):
            fk(SimpleNamespace(name=["yaw"], position=[float("nan")]))
        path.write_bytes(blob + b" ")
        with pytest.raises(ValueError, match="模型校验"):
            m.ModelPose(spec)
    finally:
        path.unlink(missing_ok=True)


@pytest.mark.parametrize("causal", [False, True])
def test_align(causal):
    idx, valid, error = lr.align([100, 200, 300], np.array([90, 170, 400]), 50, causal)
    assert valid.tolist() == (
        [True, True, False] if not causal else [False, False, False]
    )
    if causal:
        assert np.all(error[valid] <= 0)


def test_duplicate_time_rejected():
    with pytest.raises(ValueError):
        lr.align([1, 1], np.array([1]), 1)


def test_rotation_columns():
    value, good = lr.convert_pose(np.array([[1, 2, 3, 0, 0, 0, 1.0]]), np.eye(4))
    np.testing.assert_array_equal(value, [[1, 2, 3, 1, 0, 0, 0, 1, 0]])
    assert good[0]


def test_zero_quaternion_rejected():
    _, good = lr.convert_pose(np.zeros((1, 7)), np.eye(4))
    assert not good[0]


def test_discontinuity_not_compressed():
    assert lr.longest_run(np.array([True, False, True, True, False])) == slice(2, 4)


def test_outside_data_rejected(tmp_path):
    with pytest.raises(ValueError):
        lr.safe(tmp_path / "source")


def test_symlink_escape(tmp_path):
    link = DATA / ("test_link_" + uuid.uuid4().hex)
    try:
        link.symlink_to(tmp_path, target_is_directory=True)
    except OSError as exc:
        if exc.errno in (errno.EPERM, errno.EOPNOTSUPP, errno.ENOSYS):
            pytest.skip("当前exFAT不支持符号链接；路径越界测试仍执行")
        raise
    try:
        with pytest.raises(ValueError):
            lr.safe(link / "source")
    finally:
        link.unlink()


def command(step, *args, success=True):
    if step == "read_raw_data":
        args = ("--local-source", *args)
    r = subprocess.run(
        [sys.executable, str(ROOT / "src" / step / "main.py"), *map(str, args)],
        capture_output=True,
        text=True,
    )
    if success:
        assert r.returncode == 0, r.stderr
    else:
        assert r.returncode != 0
    return r


def fixture_data(task):
    import base64

    source = DATA / "test_sources" / task
    ep = source / "my-bag-26-10-23-12-00-00"
    ep.mkdir(parents=True)
    b = io.BytesIO()
    Image.new("RGB", (32, 24), (60, 120, 180)).save(b, format="JPEG")
    jpeg = base64.b64encode(b.getvalue()).decode()
    streams = {}
    rows = []
    start = 1792750000000000000
    for role in ["observations", "actions"]:
        for side in ["left", "right", "active_camera"]:
            key = f"{role}/{side}"
            streams[key] = {
                "topic": key,
                "kind": "pose",
                "frame": "base_link",
                "unit": "m",
                "source_role": "state" if role == "observations" else "action",
            }
        key = f"{role}/chassis"
        streams[key] = {
            "topic": key,
            "kind": "vector",
            "unit": "m/s,m/s,rad/s",
            "source_role": "state" if role == "observations" else "action",
        }
    for camera in ["left", "right", "active_camera"]:
        key = f"images/{camera}"
        streams[key] = {"topic": key, "kind": "image"}
    for i in range(8):
        for key, spec in streams.items():
            r = {
                "topic": key,
                "timestamp_ns": start + i * 50000000,
                "clock_domain": "robot_receive",
                "source_role": spec.get("source_role", "image"),
                "coordinate_frame": spec.get("frame", ""),
            }
            if spec["kind"] == "image":
                r["jpeg_base64"] = jpeg
            else:
                r["value"] = (
                    [
                        i * 0.01 + (0.2 if key.startswith("actions") else 0),
                        0,
                        0,
                        0,
                        0,
                        0,
                        1,
                    ]
                    if spec["kind"] == "pose"
                    else [i * 0.01, 0, 0]
                )
            rows.append(r)
    (ep / "episode_metadata.json").write_text(json.dumps({"capture_id": task}))
    (ep / "streams.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (ep / "unknown.bin").write_bytes(b"preserve this unknown data\x00")
    cfg = DATA / "config" / f"{task}_apex.json"
    cfg.write_text(
        json.dumps({"fps": 20, "clock_domain": "robot_receive", "streams": streams})
    )
    lcfg = DATA / "config" / f"{task}_lr.json"
    lcfg.write_text(
        json.dumps(
            {
                "fps": 20,
                "max_skew_ms": 50,
                "min_frames": 2,
                "frame_transforms": {},
                "allow_missing": [],
            }
        )
    )
    hs = DATA / "test_sources" / (task + "_hand")
    hp = hs / ep.name
    hp.mkdir(parents=True)
    names = json.loads((DATA / "references/info.json").read_text())["features"][
        "action"
    ]["names"]
    hands = {}
    for role in ["state", "action"]:
        for side, startidx in [("left", 32), ("right", 55)]:
            hands[f"{role}/{side}"] = {
                "timestamp_ns": [start + i * 50000000 for i in range(8)],
                "values": [[0.1 if role == "state" else 0.2] * 22 for _ in range(8)],
                "joint_names": names[startidx : startidx + 22],
                "unit": "rad",
                "clock_domain": "robot_receive",
                "source_role": role,
            }
    (hp / "sharpa.json").write_text(json.dumps({"capture_id": task, "streams": hands}))
    return source, hs, cfg, lcfg


def test_three_commands_resume_and_failure():
    task = "synthetic_" + uuid.uuid4().hex[:8]
    src, hs, cfg, lcfg = fixture_data(task)
    try:
        command(
            "read_raw_data", "--task", task, "--apex-source", src, "--sharpa-source", hs
        )
        config = json.loads(cfg.read_text())
        config["streams"]["missing"] = {"topic": "/missing", "kind": "vector"}
        cfg.write_text(json.dumps(config))
        command("apex2hdf5", "--task", task, "--config", cfg, success=False)
        output_dir = DATA / "hdf5" / task
        assert not list(output_dir.glob("*.partial")) and not list(
            output_dir.glob("*.hdf5")
        )
        assert (
            json.loads((output_dir / "progress.json").read_text())["status"]
            == "interrupted"
        )
        del config["streams"]["missing"]
        cfg.write_text(json.dumps(config))
        command("apex2hdf5", "--task", task, "--config", cfg)
        command("hdf52lerobot", "--task", task, "--config", lcfg)
        out = DATA / "lerobot" / task / "episode_00000000"
        table = pq.read_table(out / "data/chunk-000/episode_000000.parquet")
        assert (
            json.loads(table.schema.metadata[b"huggingface"])["info"]["features"][
                "observation.images.head_color"
            ]["_type"]
            == "Image"
        )
        state = np.array(table["observation.state"].to_pylist())
        action = np.array(table["action"].to_pylist())
        assert table.num_rows == 8 and state.shape == (8, 78)
        assert np.allclose(action[:, 0] - state[:, 0], 0.2)
        assert np.allclose(state[:, 32:54], 0.1) and np.allclose(action[:, 32:54], 0.2)
        assert np.array(table["validity"].to_pylist()).sum(axis=1).tolist() == [71] * 8
        before = (out / "data/chunk-000/episode_000000.parquet").stat().st_mtime_ns
        command(
            "read_raw_data", "--task", task, "--apex-source", src, "--sharpa-source", hs
        )
        command("apex2hdf5", "--task", task, "--config", cfg)
        command("hdf52lerobot", "--task", task, "--config", lcfg)
        assert (
            before == (out / "data/chunk-000/episode_000000.parquet").stat().st_mtime_ns
        )
        # Corrupt sample: fail without publishing any partial result.
        shutil.rmtree(out)
        hand = (
            DATA
            / "raw_data/sharpa_data"
            / task
            / "episode_00000000/payload/sharpa.json"
        )
        obj = json.loads(hand.read_text())
        obj["streams"]["state/left"]["clock_domain"] = "unsynced"
        hand.write_text(json.dumps(obj))
        command("hdf52lerobot", "--task", task, "--config", lcfg, success=False)
        assert not out.exists() and not list(out.parent.glob("*.partial"))
        checkpoint = json.loads((out.parent / "progress.json").read_text())
        assert checkpoint["status"] == "interrupted"
    finally:
        for path in [
            src,
            hs,
            DATA / "raw_data/apex_data" / task,
            DATA / "raw_data/sharpa_data" / task,
            DATA / "hdf5" / task,
            DATA / "lerobot" / task,
        ]:
            shutil.rmtree(path, ignore_errors=True)
        cfg.unlink()
        lcfg.unlink()


def test_mcap_custom_vector_and_h264():
    import av, h5py
    from mcap_ros2.writer import Writer

    ap = module("apex2hdf5")
    base = DATA / "test_sources" / ("mcap_" + uuid.uuid4().hex)
    payload = base / "payload"
    payload.mkdir(parents=True)
    try:
        codec = av.CodecContext.create("libx264", "w")
        codec.width = 32
        codec.height = 24
        codec.pix_fmt = "yuv420p"
        codec.options = {"preset": "ultrafast", "tune": "zerolatency"}
        with (payload / "record.mcap").open("wb") as f:
            writer = Writer(f)
            vector = writer.register_msgdef("test_msgs/msg/Vector", "float64[] value")
            image = writer.register_msgdef(
                "sensor_msgs/msg/CompressedImage", "string format\nuint8[] data"
            )
            for i in range(4):
                t = 1000000000 + i * 50000000
                writer.write_message(
                    "/vector", vector, {"value": [1.0, 2.0, 3.0]}, log_time=t
                )
                frame = av.VideoFrame.from_ndarray(
                    np.full((24, 32, 3), i * 40, dtype="uint8"), format="rgb24"
                )
                packets = codec.encode(frame)
                assert len(packets) == 1
                writer.write_message(
                    "/video",
                    image,
                    {"format": "h264", "data": bytes(packets[0])},
                    log_time=t,
                )
            writer.finish()
        cfg = {
            "fps": 20,
            "clock_domain": "robot_receive",
            "streams": {
                "observations/chassis": {
                    "topic": "/vector",
                    "kind": "vector",
                    "field": "value",
                    "source_role": "state",
                },
                "images/quad": {"topic": "/video", "kind": "image", "field": "data"},
            },
        }
        out = base / "result.hdf5"
        ap.convert(base, out, cfg)
        with h5py.File(out) as h:
            assert h.attrs["complete"] and len(h["images/quad/jpeg"]) == 4
            assert h["images/quad/timestamp_ns"][:].tolist() == [
                1000000000 + i * 50000000 for i in range(4)
            ]
            assert any(
                d.attrs["relative_path"] == "record.mcap"
                for d in h["raw_files"].values()
            )
    finally:
        shutil.rmtree(base)


def test_copy_interruption_removes_partial_and_resumes():
    raw = module("read_raw_data")
    task = "interrupt_" + uuid.uuid4().hex
    source = DATA / "test_sources" / task
    ep = source / "my-bag-26-10-23-01-00-00"
    ep.mkdir(parents=True)
    (ep / "a.bin").write_bytes(b"x" * 20)
    dest = DATA / "raw_data/apex_data" / task

    class Broken(raw.Source):
        def open(self, path):
            raise KeyboardInterrupt("simulated interruption")

    try:
        with pytest.raises(KeyboardInterrupt):
            raw.copy_task(Broken(source), dest)
        assert not (dest / "episode_00000000").exists() and not list(
            dest.glob(".*.partial")
        )
        assert (
            json.loads((dest / "copy_progress.json").read_text())["current"]["status"]
            == "interrupted"
        )
        raw.copy_task(raw.Source(source), dest)
        assert (dest / "episode_00000000/payload/a.bin").read_bytes() == b"x" * 20
    finally:
        shutil.rmtree(source)
        shutil.rmtree(dest)


@pytest.fixture
def prepared():
    task = "audit_" + uuid.uuid4().hex[:10]
    src, hs, cfg, lcfg = fixture_data(task)
    paths = [
        src,
        hs,
        DATA / "raw_data/apex_data" / task,
        DATA / "raw_data/sharpa_data" / task,
        DATA / "hdf5" / task,
        DATA / "lerobot" / task,
    ]
    try:
        command(
            "read_raw_data", "--task", task, "--apex-source", src, "--sharpa-source", hs
        )
        command("apex2hdf5", "--task", task, "--config", cfg)
        yield {
            "task": task,
            "source": src,
            "config": cfg,
            "lr_config": lcfg,
            "hdf5": paths[4] / "hdf5_00000000.hdf5",
            "hand": paths[3] / "episode_00000000/payload/sharpa.json",
            "output": paths[5] / "episode_00000000",
            "raw": paths[2] / "episode_00000000",
        }
    finally:
        for path in paths:
            shutil.rmtree(path, ignore_errors=True)
        cfg.unlink(missing_ok=True)
        lcfg.unlink(missing_ok=True)


def export(data, success=True):
    return command(
        "hdf52lerobot",
        "--task",
        data["task"],
        "--config",
        data["lr_config"],
        success=success,
    )


def test_precise_clock_mapping_and_reorder():
    epoch = 1792750000000000000
    np.testing.assert_array_equal(
        lr.map_clock(
            [epoch + 1, epoch + 3],
            "hand",
            "robot",
            {
                "source_origin_ns": epoch,
                "target_origin_ns": epoch + 100,
                "scale": 1,
                "target_domain": "robot",
            },
        ),
        [epoch + 101, epoch + 103],
    )
    for bad in [[1.0, 2.0], [True, False], [3, 2], [1, 1], [-1, 2]]:
        with pytest.raises(ValueError):
            lr.timestamps(bad)
    for scale in [0, -1, "NaN", "Infinity"]:
        with pytest.raises((ValueError, OverflowError)):
            lr.map_clock(
                [1, 2],
                "hand",
                "robot",
                {
                    "source_origin_ns": 0,
                    "target_origin_ns": 0,
                    "scale": scale,
                    "target_domain": "robot",
                },
            )
    with pytest.raises(ValueError):
        lr.map_clock([1, 2], "hand", "robot")
    with pytest.raises(ValueError):
        lr.map_clock(
            [1, 2],
            "hand",
            "robot",
            {
                "source_origin_ns": 0,
                "target_origin_ns": 2**63 - 1,
                "scale": 1,
                "target_domain": "robot",
            },
        )


def test_causal_action_and_alignment_trace(prepared):
    obj = json.loads(prepared["hand"].read_text())
    for key, stream in obj["streams"].items():
        if key.startswith("action/"):
            stream["timestamp_ns"] = [t + 10_000_000 for t in stream["timestamp_ns"]]
            stream["values"] = [[0.2 + i] * 22 for i in range(8)]
    prepared["hand"].write_text(json.dumps(obj))
    export(prepared)
    table = pq.read_table(prepared["output"] / "data/chunk-000/episode_000000.parquet")
    actions = np.array(table["action"].to_pylist())
    assert table.num_rows == 7
    np.testing.assert_allclose(actions[:, 32], np.arange(7) + 0.2)
    trace = json.loads((prepared["output"] / "meta/source_timestamps.json").read_text())
    assert trace["aligned_sources"]["action/left"]["source_index"] == list(range(7))
    assert all(
        a <= t
        for a, t in zip(
            trace["aligned_sources"]["action/left"]["timestamp_ns"],
            trace["timestamp_ns"],
        )
    )
    assert (
        json.loads((prepared["output"] / "meta/cleaning_report.json").read_text())[
            "retained_max_spread_ns"
        ]
        == 40_000_000
    )


def test_pairwise_skew_not_just_grid_distance(prepared):
    obj = json.loads(prepared["hand"].read_text())
    for key, stream in obj["streams"].items():
        delta = 20_000_000 if key.startswith("state/") else 10_000_000
        stream["timestamp_ns"] = [t + delta for t in stream["timestamp_ns"]]
    prepared["hand"].write_text(json.dumps(obj))
    result = export(prepared, False)
    assert "有效连续帧" in result.stderr
    assert not prepared["output"].exists()


@pytest.mark.parametrize("fault", ["capture", "role", "clock", "frame", "external"])
def test_semantic_mismatches_rejected(prepared, fault):
    import h5py

    if fault in ["capture", "role"]:
        obj = json.loads(prepared["hand"].read_text())
        if fault == "capture":
            obj["capture_id"] = "different_recording"
        else:
            obj["streams"]["state/left"]["source_role"] = "action"
        prepared["hand"].write_text(json.dumps(obj))
    else:
        with h5py.File(prepared["hdf5"], "a") as h:
            if fault == "clock":
                h["observations/left"].attrs["clock_domain"] = "other_clock"
            elif fault == "frame":
                h["observations/left"].attrs["coordinate_frame"] = "unknown_frame"
            else:
                h["outside"] = h5py.ExternalLink("/not/allowed.hdf5", "/")
    result = export(prepared, False)
    assert not prepared["output"].exists() and not list(
        prepared["output"].parent.glob(".*.partial")
    )
    assert any(
        word in result.stderr
        for word in ["capture_id", "source_role", "时钟域", "标定", "链接"]
    )


def test_resume_detects_corrupted_outputs(prepared):
    export(prepared)
    p = prepared["output"] / "data/chunk-000/episode_000000.parquet"
    p.write_bytes(b"corrupt output")
    result = export(prepared, False)
    assert "损坏" in result.stderr
    with prepared["hdf5"].open("ab") as f:
        f.write(b"corrupt")
    result = command(
        "apex2hdf5",
        "--task",
        prepared["task"],
        "--config",
        prepared["config"],
        success=False,
    )
    assert "损坏" in result.stderr
    assert (
        json.loads((prepared["hdf5"].parent / "progress.json").read_text())["status"]
        == "interrupted"
    )


def test_identical_state_action_source_and_clock_backstep_rejected(prepared):
    ap = module("apex2hdf5")
    cfg = json.loads(prepared["config"].read_text())
    cfg["streams"]["actions/left"]["topic"] = cfg["streams"]["observations/left"][
        "topic"
    ]
    tmp = prepared["hdf5"].parent / "test.partial"
    with pytest.raises(ValueError, match="同一源字段"):
        ap.convert(prepared["raw"], tmp, cfg)
    cfg = json.loads(prepared["config"].read_text())
    path = prepared["raw"] / "payload/streams.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    points = [r for r in rows if r["topic"] == "observations/left"]
    points[2]["timestamp_ns"] = points[0]["timestamp_ns"] - 1
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    with pytest.raises(ValueError, match="回退"):
        ap.convert(prepared["raw"], tmp, cfg)


def test_nonhand_profile_omits_hand_dimensions_and_preserves_extra_sources(prepared):
    cfg = json.loads(prepared["lr_config"].read_text())
    cfg.update(profile="apex_nonhand", allow_missing=[], extra_streams={
        "observation.native_body": {"path": "observations/chassis",
            "names": ["a", "b", "c"], "source_stage": "measured_feedback"},
        "action.native_body": {"path": "actions/chassis",
            "names": ["a", "b", "c"], "source_stage": "controller_output"}})
    prepared["lr_config"].write_text(json.dumps(cfg))
    prepared["hand"].unlink()
    export(prepared)
    out = prepared["output"]
    info = json.loads((out / "meta/info.json").read_text())
    assert info["features"]["action"]["shape"] == [27]
    assert info["features"]["observation.state"]["shape"] == [27]
    assert info["egoav78_wire_compatible"] is False
    assert all(info["action_mask"])
    table = pq.read_table(out / "data/chunk-000/episode_000000.parquet")
    assert np.array(table["validity"].to_pylist()).all()
    assert table["action.native_body"].to_pylist() == table["action.chassis"].to_pylist()
    report = json.loads((out / "meta/cleaning_report.json").read_text())
    assert report["extra_streams"]["action.native_body"]["source_stage"] == "controller_output"


def test_complete_output_types_stats_and_sources(prepared):
    import h5py

    export(prepared)
    out = prepared["output"]
    info = json.loads((out / "meta/info.json").read_text())
    table = pq.read_table(out / "data/chunk-000/episode_000000.parquet")
    assert set(table.column_names) == set(info["features"])
    for key, spec in info["features"].items():
        if spec["dtype"] == "image":
            for item in table[key].to_pylist():
                with Image.open(io.BytesIO(item["bytes"])) as image:
                    assert [3, image.height, image.width] == spec["shape"]
        elif spec["shape"] != [1]:
            assert table.schema.field(key).type.list_size == spec["shape"][0]
    statistics = json.loads((out / "meta/stats.json").read_text())
    for key in ["observation.state", "action", "observation.chassis", "action.chassis"]:
        values = np.array(table[key].to_pylist())
        assert statistics[key]["count"] == [8]
        np.testing.assert_allclose(statistics[key]["mean"], values.mean(0), atol=1e-7)
        np.testing.assert_allclose(statistics[key]["std"], values.std(0), atol=1e-7)
    with h5py.File(prepared["hdf5"]) as h:
        assert h.attrs["capture_id"] == prepared["task"]
        assert h["observations/left"].attrs["source_role"] == "state"
        assert h["actions/left"].attrs["source_role"] == "action"
        np.testing.assert_array_equal(
            h["observations/left/source_timestamp_ns"],
            h["observations/left/timestamp_ns"],
        )
        unknown = [
            v
            for v in h["raw_files"].values()
            if v.attrs["relative_path"] == "unknown.bin"
        ][0]
        assert unknown[:].tobytes() == b"preserve this unknown data\x00"


def test_shard_order_is_log_time_not_filename():
    from mcap_ros2.writer import Writer

    ap = module("apex2hdf5")
    base = DATA / "test_sources" / ("shards_" + uuid.uuid4().hex)
    base.mkdir(parents=True)
    try:
        for name, times in [
            ("record_10.mcap", [300, 500]),
            ("record_2.mcap", [100, 400]),
        ]:
            with (base / name).open("wb") as f:
                w = Writer(f)
                schema = w.register_msgdef("test_msgs/msg/Vector", "float64[] value")
                for t in times:
                    w.write_message("/v", schema, {"value": [float(t)]}, log_time=t)
                w.finish()
        assert [t for _, t, _ in ap.messages(base, ["/v"])] == [100, 300, 400, 500]
    finally:
        shutil.rmtree(base)


def test_copy_preserves_empty_directories_and_scans_sequentially():
    raw = module("read_raw_data")
    task = "serial_" + uuid.uuid4().hex
    base = DATA / "test_sources" / task
    dest = DATA / "raw_data/apex_data" / task
    names = ["my-bag-26-10-23-01-00-00", "my-bag-26-10-23-02-00-00"]
    for name in names:
        (base / name / "empty").mkdir(parents=True)
        (base / name / "file").write_bytes(b"test")

    class Ordered(raw.Source):
        def walk(self, p):
            if p.endswith(names[1]):
                assert (dest / "episode_00000000/manifest.json").exists()
            yield from super().walk(p)

    try:
        raw.copy_task(Ordered(base), dest)
        assert (dest / "episode_00000000/payload/empty").is_dir()
        progress = json.loads((dest / "copy_progress.json").read_text())
        assert all(v["status"] == "complete" for v in progress["episodes"].values())
    finally:
        shutil.rmtree(base)
        shutil.rmtree(dest)


@pytest.mark.parametrize("step", ["apex2hdf5", "hdf52lerobot"])
@pytest.mark.parametrize("signal_name", ["SIGTERM", "SIGKILL"])
def test_real_process_interruption_and_restart(prepared, step, signal_name):
    import signal, time

    marker = prepared["hdf5"].parent / "signal-ready"
    if step == "apex2hdf5":
        prepared["hdf5"].unlink()
        prepared["hdf5"].with_suffix(".commit.json").unlink()
        function = "convert"
        config = prepared["config"]
        pos = 1
    else:
        function = "build"
        config = prepared["lr_config"]
        pos = 2
    program = f"""import importlib.util,sys,time
from pathlib import Path
spec=importlib.util.spec_from_file_location('pipeline',sys.argv[1]); m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
def pause(*args,**kwargs):
 p=args[{pos}]
 if {step!r}=='apex2hdf5': p.write_bytes(b'incomplete hdf5')
 else:
  p.mkdir(parents=True); (p/'incomplete.parquet').write_bytes(b'incomplete parquet')
 Path({str(marker)!r}).write_text('ready')
 time.sleep(30)
m.{function}=pause
sys.argv=[sys.argv[1],*sys.argv[2:]]
m.main()
"""
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            program,
            str(ROOT / "src" / step / "main.py"),
            "--task",
            prepared["task"],
            "--config",
            str(config),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while (
            not marker.exists()
            and time.monotonic() < deadline
            and process.poll() is None
        ):
            time.sleep(0.01)
        assert marker.exists(), process.communicate(timeout=1)
        process.send_signal(getattr(signal, signal_name))
        process.communicate(timeout=5)
        assert process.returncode != 0
        parent = (
            prepared["hdf5"].parent
            if step == "apex2hdf5"
            else prepared["output"].parent
        )
        partials = list(parent.glob("*.partial")) + list(parent.glob(".*.partial"))
        if signal_name == "SIGTERM":
            assert not partials
        else:
            assert partials
        command(step, "--task", prepared["task"], "--config", config)
        assert not list(parent.glob("*.partial")) and not list(
            parent.glob(".*.partial")
        )
        assert (
            json.loads((parent / "progress.json").read_text())["status"] == "complete"
        )
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        marker.unlink(missing_ok=True)


def test_header_clock_requires_mapping():
    from types import SimpleNamespace

    ap = module("apex2hdf5")
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=7))
    )
    t = ap.stamp(message, 1792750000000000000, "header")
    with pytest.raises(ValueError):
        ap.map_clock([t], "chassis_clock", "robot_receive")
    np.testing.assert_array_equal(
        ap.map_clock(
            [t],
            "chassis_clock",
            "robot_receive",
            {
                "source_origin_ns": 1000000000,
                "target_origin_ns": 1792750000000000000,
                "scale": 1,
                "target_domain": "robot_receive",
            },
        ),
        [1792750000000000007],
    )


@pytest.mark.parametrize("signal_name", ["SIGTERM", "SIGKILL"])
def test_raw_copy_real_signal_and_restart(prepared, signal_name):
    import signal, time

    raw_task = prepared["raw"].parent
    shutil.rmtree(raw_task)
    marker = prepared["hdf5"].parent / "copy-ready"
    program = f"""import importlib.util,sys,time
from pathlib import Path
spec=importlib.util.spec_from_file_location('pipeline',sys.argv[1]);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
original=m.Source.open
class Slow:
 def __init__(self,f): self.f=f;self.count=0
 def __enter__(self): return self
 def __exit__(self,*args): self.f.close()
 def read(self,*args):
  self.count+=1
  if self.count==2:
   Path({str(marker)!r}).write_text('ready')
   time.sleep(30)
  return self.f.read(*args)
m.Source.open=lambda self,path:Slow(original(self,path))
sys.argv=[sys.argv[1],*sys.argv[2:]]
m.main()
"""
    process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            program,
            str(ROOT / "src/read_raw_data/main.py"),
            "--local-source",
            "--task",
            prepared["task"],
            "--apex-source",
            str(prepared["source"]),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while (
            not marker.exists()
            and time.monotonic() < deadline
            and process.poll() is None
        ):
            time.sleep(0.01)
        assert marker.exists(), process.communicate(timeout=1)
        process.send_signal(getattr(signal, signal_name))
        process.communicate(timeout=5)
        assert not prepared["raw"].exists()
        if signal_name == "SIGTERM":
            assert not list(raw_task.glob(".*.partial"))
        else:
            assert list(raw_task.glob(".*.partial"))
        command(
            "read_raw_data",
            "--task",
            prepared["task"],
            "--apex-source",
            prepared["source"],
        )
        assert prepared["raw"].exists() and not list(raw_task.glob(".*.partial"))
        assert (
            json.loads((raw_task / "copy_progress.json").read_text())["current"][
                "status"
            ]
            == "complete"
        )
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        marker.unlink(missing_ok=True)


def test_missing_mount_never_creates_system_directory(monkeypatch, tmp_path):
    mount = tmp_path / "disconnected-drive"
    for step in ["read_raw_data", "apex2hdf5", "hdf52lerobot"]:
        m = module(step)
        monkeypatch.setattr(m, "STORAGE_MOUNT", mount)
        monkeypatch.setattr(m, "DATA", mount / "apex_n_sharpa/data")
        with pytest.raises(RuntimeError, match="未挂载"):
            m.lock("test")
        assert not mount.exists()


def test_wrong_disk_serial_rejected(monkeypatch):
    m = module("hdf52lerobot")

    def output(args, **kwargs):
        return "/dev/fake\n" if args[0] == "findmnt" else "OTHER-DISK\n"

    monkeypatch.setattr(subprocess, "check_output", output)
    with pytest.raises(RuntimeError, match="不是指定WD"):
        m.ensure_storage()


@pytest.mark.parametrize("local", [False, True])
def test_read_source_default_and_explicit_local_mode(monkeypatch, local):
    m = module("read_raw_data")
    calls = []

    class FakeSource:
        def __init__(self, path, ssh, timezone):
            calls.append((path, ssh))

        def close(self):
            pass

    monkeypatch.setattr(m, "Source", FakeSource)
    monkeypatch.setattr(m, "copy_task", lambda source, dest: None)
    monkeypatch.setattr(
        sys,
        "argv",
        ["read_raw_data", "--task", "test_connection_defaults"]
        + (["--local-source"] if local else []),
    )
    m.main()
    assert calls == [
        ("/home/user/gento_replay_bags", None if local else "user@6.6.7.100")
    ]


def test_all_commands_use_external_data_root():
    for step in ["read_raw_data", "apex2hdf5", "hdf52lerobot"]:
        m = module(step)
        assert m.DATA == DATA and m.DATA != ROOT / "data"
        with pytest.raises(ValueError):
            m.safe(ROOT / "data/config/apex.json")
