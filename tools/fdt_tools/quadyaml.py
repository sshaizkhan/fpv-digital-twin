"""Write a fitted value back into config/quad.yaml, with its provenance.

Deliberately a surgical TEXT edit rather than a YAML load-and-dump. quad.yaml
carries a great deal of explanatory comment -- why each placeholder is what it
is, how to measure it -- and a round-trip through a YAML library would throw
all of it away. The file is the project's record of what is known and what is
guessed; losing that would cost more than the convenience is worth.

Every write also flips `measured: true` and replaces `source:`, so a fitted
value can never masquerade as a hand-entered one, or vice versa.
"""
from __future__ import annotations

import re
from datetime import date
from pathlib import Path


class QuadYamlError(RuntimeError):
    pass


def _find_block(text: str, dotted: str) -> tuple[int, int, int]:
    """Locate a parameter block, returning (start, end, indent of its keys).

    `dotted` is e.g. "motors.model.time_constant". Each segment must appear, in
    order, at strictly increasing indentation, so a leaf name that occurs under
    several parents cannot be confused.
    """
    lines = text.splitlines(keepends=True)
    segments = dotted.split(".")

    offset = 0
    parent_indent = -1
    search_from = 0
    for depth, segment in enumerate(segments):
        pattern = re.compile(rf"^(\s*){re.escape(segment)}:")
        found = None
        for index in range(search_from, len(lines)):
            match = pattern.match(lines[index])
            if not match:
                continue
            indent = len(match.group(1))
            if indent <= parent_indent:
                if depth > 0:
                    break  # left the parent's block without finding it
                continue
            found = (index, indent)
            break
        if found is None:
            raise QuadYamlError(f"{dotted}: no '{segment}:' found in quad.yaml")
        search_from, parent_indent = found[0] + 1, found[1]
        offset = found[0]

    key_indent = parent_indent
    end = len(lines)
    for index in range(offset + 1, len(lines)):
        stripped = lines[index].strip()
        if not stripped:
            continue
        indent = len(lines[index]) - len(lines[index].lstrip())
        if indent <= key_indent:
            end = index
            break
    return offset, end, key_indent


def update_parameter(path: str | Path, dotted: str, value: float, source: str,
                     measured: bool = True) -> str:
    """Set value/measured/source on one parameter block. Returns the old value."""
    path = Path(path)
    text = path.read_text()
    start, end, key_indent = _find_block(text, dotted)
    lines = text.splitlines(keepends=True)
    field_indent = " " * (key_indent + 2)

    old_value = None
    saw = {"value": False, "measured": False, "source": False}
    out = list(lines[: start + 1])

    index = start + 1
    while index < end:
        line = lines[index]
        stripped = line.strip()
        if stripped.startswith("value:"):
            old_value = stripped.split(":", 1)[1].strip()
            out.append(f"{field_indent}value: {value!r}\n".replace("'", ""))
            saw["value"] = True
        elif stripped.startswith("measured:"):
            out.append(f"{field_indent}measured: {'true' if measured else 'false'}\n")
            saw["measured"] = True
        elif stripped.startswith("source:"):
            # Drop any continuation lines of the old source string.
            index += 1
            while index < end:
                nxt = lines[index].strip()
                if (not nxt) or re.match(r"^[A-Za-z_][A-Za-z0-9_]*:", nxt) or nxt.startswith("#"):
                    break
                index += 1
            out.append(f'{field_indent}source: "{source}"\n')
            saw["source"] = True
            continue
        else:
            out.append(line)
        index += 1

    missing = [k for k, v in saw.items() if not v]
    if missing:
        raise QuadYamlError(f"{dotted}: block is missing {missing}; refusing to write a partial update")

    out.extend(lines[end:])
    path.write_text("".join(out))
    return old_value if old_value is not None else "?"


def fitted_source(log_name: str, detail: str) -> str:
    """A source string that says where the number came from and when."""
    return f"fitted from {log_name} on {date.today().isoformat()}; {detail}"
