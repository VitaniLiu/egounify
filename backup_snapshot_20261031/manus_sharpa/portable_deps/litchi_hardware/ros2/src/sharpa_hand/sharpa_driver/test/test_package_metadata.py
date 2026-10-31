import runpy
from pathlib import Path
from unittest.mock import patch


def test_package_requires_sharpa_hal_with_firmware_initialization_fix() -> None:
    setup_kwargs = {}
    setup_path = Path(__file__).parents[1] / "setup.py"

    with patch("setuptools.setup", side_effect=lambda **kwargs: setup_kwargs.update(kwargs)):
        runpy.run_path(str(setup_path), run_name="__main__")

    assert "litchi-hardware[sharpa]>=0.1.4" in setup_kwargs["install_requires"]
