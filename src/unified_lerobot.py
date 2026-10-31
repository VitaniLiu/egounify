"""Read-only adapter for original episode IDs using LeRobot 0.3.3 / format v2.1.

The conversion environment does not import this optional training adapter.
Import it only in a training environment with lerobot==0.3.3.
"""
import json
from pathlib import Path
from importlib.metadata import version


from lerobot.datasets.lerobot_dataset import LeRobotDataset
from original_episode_index import OriginalEpisodeIndexMixin

class OriginalEpisodeDataset(OriginalEpisodeIndexMixin, LeRobotDataset):
    pass


def load_episode(root, *, delta_timestamps=None, image_transforms=None, allow_diagnostic=False):
    root = Path(root).resolve()
    info = json.loads((root / 'meta/info.json').read_text())
    capture = json.loads((root / 'meta/capture.json').read_text())
    eid = info.get('source_episode_index')
    if (info.get('profile') != 'tianji_unified78' or info.get('codebase_version') != 'v2.1'
            or type(eid) is not int or info.get('episode_ids') != [eid]
            or info.get('total_episodes') != 1 or capture.get('episode_index') != eid
            or info.get('capture_id') != capture.get('capture_id')
            or info.get('diagnostic') != capture.get('diagnostic')):
        raise ValueError('Unified original-episode metadata required')
    if info.get('diagnostic') and not allow_diagnostic:
        raise ValueError('Diagnostic data rejected for training')
    if version('lerobot') != '0.3.3':
        raise ValueError('This v2.1 reader is validated with lerobot==0.3.3')
    return OriginalEpisodeDataset(
        repo_id=f'local/episode_{eid:08d}', root=root, episodes=[eid],
        delta_timestamps=delta_timestamps, image_transforms=image_transforms,
        download_videos=False,
    )
