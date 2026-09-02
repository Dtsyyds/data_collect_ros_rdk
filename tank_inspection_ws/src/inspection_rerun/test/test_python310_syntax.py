import ast
from pathlib import Path


def test_package_parses_as_python310():
    package = Path(__file__).parents[1] / "inspection_rerun"
    for source in package.glob("*.py"):
        ast.parse(source.read_text(encoding="utf-8"), filename=str(source), feature_version=(3, 10))
