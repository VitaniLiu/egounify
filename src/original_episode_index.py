"""Map original episode IDs to in-memory row bounds without changing IDs."""

class OriginalEpisodeIndexMixin:
    def _get_query_indices(self, idx, ep_idx):
        # LeRobot 0.3.3 indexes a dense bounds tensor using the recorded ID.
        # Translate only this lookup; never alter parquet IDs or returned items.
        try:
            position = self.episodes.index(ep_idx)
        except ValueError as exc:
            raise ValueError(f"Unknown original episode ID: {ep_idx}") from exc
        return super()._get_query_indices(idx, position)

    def add_frame(self, *args, **kwargs):
        raise RuntimeError("Certified original episodes are read-only")

    def save_episode(self, *args, **kwargs):
        raise RuntimeError("Certified original episodes are read-only")

