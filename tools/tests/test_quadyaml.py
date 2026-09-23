"""quad.yaml writeback: it must update the value AND its provenance, preserve
the file's comments, and refuse anything it cannot do cleanly."""
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from fdt_tools import quadyaml  # noqa: E402


def block_of(text: str, key: str) -> str:
    """Just the named parameter block, stopping at the next key at its indent.

    Slicing a fixed number of characters runs into the NEXT parameter, whose
    untouched PLACEHOLDER source then looks like a failed write.
    """
    start = text.index(f"    {key}:")
    lines = text[start:].splitlines(keepends=True)
    out = [lines[0]]
    for line in lines[1:]:
        if line.strip() and not line.startswith("      "):
            break
        out.append(line)
    return "".join(out)

REPO = Path(__file__).resolve().parent.parent.parent


@pytest.fixture
def yaml_copy(tmp_path):
    dst = tmp_path / "quad.yaml"
    shutil.copy(REPO / "config" / "quad.yaml", dst)
    return dst


def test_updates_value_measured_and_source(yaml_copy):
    old = quadyaml.update_parameter(yaml_copy, "motors.model.time_constant", 0.0312,
                                    "fitted from LOG00042.BFL; R2=0.97")
    text = yaml_copy.read_text()
    assert old == "0.025"
    block = block_of(text, "time_constant")
    assert "value: 0.0312" in block
    assert "measured: true" in block
    assert "fitted from LOG00042.BFL" in block
    assert "PLACEHOLDER" not in block, "the old source must be replaced, not appended"
    # And the NEXT block must be untouched.
    assert "PLACEHOLDER" in block_of(text, "rotor_inertia")


def test_comments_survive(yaml_copy):
    before = yaml_copy.read_text()
    comment_lines = [ln for ln in before.splitlines() if ln.strip().startswith("#")]
    quadyaml.update_parameter(yaml_copy, "motors.model.load_factor", 0.81, "fitted")
    after_comments = [ln for ln in yaml_copy.read_text().splitlines() if ln.strip().startswith("#")]
    # The only comments that may vanish are ones inside the rewritten block.
    assert len(after_comments) >= len(comment_lines) - 6
    assert "# ---------------------------------------------------------------------------" in yaml_copy.read_text()


def test_the_result_is_still_loadable_by_the_cpp_schema(yaml_copy):
    """A writeback that produces YAML the C++ loader rejects is worse than none."""
    import subprocess
    quadyaml.update_parameter(yaml_copy, "motors.model.thrust_coeff", 3.1e-6, "fitted from a log")
    dump = REPO / "build" / "make" / "physics" / "fdt_config_dump"
    if not dump.exists():
        pytest.skip("fdt_config_dump not built")
    result = subprocess.run([str(dump), str(yaml_copy)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "3.1e-06" in result.stdout or "3.100e-06" in result.stdout


def test_unknown_parameter_is_an_error(yaml_copy):
    with pytest.raises(quadyaml.QuadYamlError):
        quadyaml.update_parameter(yaml_copy, "motors.model.does_not_exist", 1.0, "x")


def test_nested_names_are_not_confused(yaml_copy):
    """`units` appears in every block; a dotted path must resolve by parent."""
    quadyaml.update_parameter(yaml_copy, "aero.induced_drag_k", 0.02, "fitted")
    text = yaml_copy.read_text()
    induced = text[text.index("  induced_drag_k:"):]
    assert "value: 0.02" in induced[:300]
    # The unrelated motors block must be untouched.
    assert "value: 2.93e-6" in text or "value: 2.93e-06" in text


def test_fitted_source_records_the_log_and_date(yaml_copy):
    source = quadyaml.fitted_source("LOG00007.BFL", "R2=0.99, n=12000")
    assert "LOG00007.BFL" in source
    assert "R2=0.99" in source
    assert source.count("-") >= 2, "should carry an ISO date"
