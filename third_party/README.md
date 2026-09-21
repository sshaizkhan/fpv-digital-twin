# third_party/

Phase 2 adds `betaflight` here as a git submodule, pinned to the tag matching
the firmware on the real FC:

```sh
git submodule add https://github.com/betaflight/betaflight.git third_party/betaflight
git -C third_party/betaflight checkout <tag from config/quad.yaml: firmware.betaflight_git_tag>
```

That tag is `null` in `config/quad.yaml` right now — the firmware version has
not been read off the real quad yet. See `docs/sitl_interface.md`.

Nothing in this directory is modified. The SITL target is built from the
unmodified firmware; any patch needed to make it build on macOS gets recorded in
`docs/sitl_interface.md` §6 rather than committed into the submodule.
