import runpy
from pathlib import Path
from unittest.mock import patch


def test_package_requires_supported_official_wuji_sdk() -> None:
    setup_kwargs = {}
    setup_path = Path(__file__).parents[1] / "setup.py"

    with patch(
        "setuptools.setup", side_effect=lambda **kwargs: setup_kwargs.update(kwargs)
    ):
        runpy.run_path(str(setup_path), run_name="__main__")

    assert "wuji-sdk>=2026.8.3,<2026.9" in setup_kwargs["install_requires"]
