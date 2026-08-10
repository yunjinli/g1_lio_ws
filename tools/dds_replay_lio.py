"""Replays a tools/dds_recorder.py capture back onto DDS, paced to the
original recording timestamps -- the mirror image of dds_recorder.py, so
anything that subscribes live (viz/rerun/viz.py chief among them) sees it
exactly as if it were a live robot, with zero consumer-side changes needed.

Message reconstruction is generic (see from_dict below): every field is
walked via typing.get_type_hints on the IDL dataclass itself rather than
hand-writing a converter per message type, the same "read the shape off the
message/class instead of hardcoding it" pattern dds_recorder.py's own
_json_default and viz.py's lidar_points already use. The one special case is
TOPICS's binary_field (currently just PointCloud2_.data): dds_recorder.py
split that out to a sibling .bin file per message instead of inlining it as
JSON, so it's read back from disk and spliced in before reconstruction.

Defaults to $G1_DOMAIN_ID/$G1_NETWORK_INTERFACE, same convention as
dds_recorder.py -- almost always run under `-e sim` (isolated domain 1/lo),
so replayed traffic can never collide with a real robot on domain 0, and
`-e sim python viz/rerun/viz.py` in a second terminal picks it up unmodified.

Usage:
    pixi run -e sim python tools/dds_replay.py --in recordings/my_session
    # in another terminal, same domain:
    pixi run -e sim python viz/rerun/viz.py
"""
import argparse
import dataclasses
import json
import os
import sys
import time
import typing
from collections import abc

import unitree_sdk2py
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PoseStamped_
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from g1_layout import LIDAR_IMU_TOPIC, LIDAR_TOPIC, VICON_PELVIS_TOPIC
from sensor_msgs_imu import Imu_

TOPICS = {
    "lowstate": ("rt/lowstate", LowState_, None),
    "vicon_pelvis": (VICON_PELVIS_TOPIC, PoseStamped_, None),
    "lidar": (LIDAR_TOPIC, PointCloud2_, "data"),
    "lidar_imu": (LIDAR_IMU_TOPIC, Imu_, None),
}


def _type_hints(cls):
    # get_type_hints resolves each base's string annotations against that
    # base's OWN module globals -- and these IDL classes annotate fields
    # with their fully-dotted path (e.g. "unitree_sdk2py.idl...Vector3_"),
    # which fails to eval unless `unitree_sdk2py`/`inspire_sdkpy` are
    # themselves bound names in that namespace, not just importable.
    ns = dict(vars(sys.modules[cls.__module__]))
    ns.setdefault("unitree_sdk2py", unitree_sdk2py)
    return typing.get_type_hints(cls, globalns=ns, include_extras=True)


def _unwrap(hint):
    return typing.get_args(hint)[0] if typing.get_origin(hint) is typing.Annotated else hint


def from_dict(cls, data):
    """Recursively rebuild an IDL dataclass instance from the dict shape
    dataclasses.asdict(msg) (dds_recorder.py's own encoder) produces --
    scalars and plain lists of scalars pass through untouched (this also
    covers the lidar `data` byte payload spliced back in by the caller, and
    the fixed-axis reserve/version arrays), only sequence-of-dataclass
    fields (e.g. LowState_.motor_state) need per-element reconstruction."""
    kwargs = {}
    for name, hint in _type_hints(cls).items():
        hint = _unwrap(hint)
        value = data[name]
        origin = typing.get_origin(hint)
        if origin in (list, abc.Sequence):
            elem = _unwrap(typing.get_args(hint)[0])
            if dataclasses.is_dataclass(elem):
                value = [from_dict(elem, v) for v in value]
        elif dataclasses.is_dataclass(hint):
            value = from_dict(hint, value)
        kwargs[name] = value
    return cls(**kwargs)


def load_records(in_dir, keys):
    """[(t, key, record_dict_without_t), ...] merged across every requested
    topic's .jsonl and sorted into one global timeline -- dds_recorder.py
    stamps every record with the same time.time() clock regardless of
    topic, so a plain global sort reproduces the original cross-topic
    interleaving (e.g. lidar at ~9.5Hz interleaved with lowstate at
    hundreds of Hz) rather than replaying one topic fully before the next."""
    records = []
    for key in keys:
        path = os.path.join(in_dir, f"{key}.jsonl")
        with open(path) as f:
            for line in f:
                record = json.loads(line)
                t = record.pop("t")
                records.append((t, key, record))
    records.sort(key=lambda item: item[0])
    return records


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="in_dir", required=True, help="Directory written by tools/dds_recorder.py --out")
    ap.add_argument(
        "--domain-id", type=int, default=None,
        help="DDS domain; defaults to $G1_DOMAIN_ID (set by -e real/-e sim/-e viz/-e ros2)",
    )
    ap.add_argument(
        "--interface", default=None,
        help="DDS network interface; defaults to $G1_NETWORK_INTERFACE (set by -e real/-e sim/-e viz/-e ros2)",
    )
    ap.add_argument(
        "--topics", default="all",
        help=f"Comma-separated keys to replay, or 'all' (default). Choices: {', '.join(TOPICS)}",
    )
    ap.add_argument("--rate", type=float, default=1.0, help="Playback speed multiplier (default 1.0 = real-time)")
    ap.add_argument("--loop", action="store_true", help="Replay the capture on repeat instead of exiting once")
    args = ap.parse_args()

    available = {
        key for key in TOPICS
        if os.path.exists(os.path.join(args.in_dir, f"{key}.jsonl"))
    }
    keys = list(available) if args.topics == "all" else args.topics.split(",")
    unknown = [k for k in keys if k not in TOPICS]
    if unknown:
        ap.error(f"unknown topic key(s) {unknown}; choices are {list(TOPICS)}")
    missing = [k for k in keys if k not in available]
    if missing:
        ap.error(f"no {args.in_dir}/<key>.jsonl for {missing} -- were they actually recorded?")
    if not keys:
        ap.error(f"no recorded topics found under {args.in_dir}")

    domain_id = args.domain_id if args.domain_id is not None else os.environ.get("G1_DOMAIN_ID")
    interface = args.interface if args.interface is not None else os.environ.get("G1_NETWORK_INTERFACE")
    if domain_id is None or not interface:
        ap.error(
            "no --domain-id/--interface given and $G1_DOMAIN_ID/$G1_NETWORK_INTERFACE aren't set -- "
            "run under -e real/-e sim/-e viz/-e ros2, or pass both explicitly"
        )
    domain_id = int(domain_id)

    print(f"[dds_replay] loading {keys} from {args.in_dir} ...")
    records = load_records(args.in_dir, keys)
    if not records:
        ap.error("no records loaded -- recording directory is empty")
    duration = records[-1][0] - records[0][0]
    print(f"[dds_replay] {len(records)} records spanning {duration:.1f}s, replaying at {args.rate}x")

    ChannelFactoryInitialize(domain_id, interface)
    publishers = {}
    for key in keys:
        topic, cls, _ = TOPICS[key]
        pub = ChannelPublisher(topic, cls)
        pub.Init()
        publishers[key] = pub
        print(f"[dds_replay] replaying {args.in_dir}/{key}.jsonl -> {topic}")

    print("[dds_replay] Ctrl+C to stop")
    try:
        while True:
            t0 = records[0][0]
            wall0 = time.perf_counter()
            counts = {key: 0 for key in keys}
            last_report = wall0
            for t, key, record in records:
                target = wall0 + (t - t0) / args.rate
                sleep_for = target - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)

                topic, cls, binary_field = TOPICS[key]
                record = dict(record)
                if binary_field:
                    with open(os.path.join(args.in_dir, record.pop("data_file")), "rb") as f:
                        record[binary_field] = f.read()
                publishers[key].Write(from_dict(cls, record))
                counts[key] += 1

                now = time.perf_counter()
                if now - last_report >= 2.0:
                    print(f"[dds_replay] {', '.join(f'{k}={v}' for k, v in counts.items())}")
                    last_report = now
            print(f"[dds_replay] pass complete: {', '.join(f'{k}={v}' for k, v in counts.items())}")
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        for pub in publishers.values():
            pub.Close()
        print("[dds_replay] stopped")


if __name__ == "__main__":
    main()
