// fdt_config_dump - load config/quad.yaml, validate it, print what it says and
// which of its numbers are still guesses.
//
//   fdt_config_dump [path/to/quad.yaml] [--list-unmeasured] [--strict]
//
// --list-unmeasured prints only the dotted paths, one per line, for scripting.
// --strict exits non-zero if any parameter is unmeasured; useful once the real
// measurements are in and you want CI to keep them in.

#include "fdt/quad_config.hpp"

#include <cstring>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string path = "config/quad.yaml";
  bool list_only = false;
  bool strict = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--list-unmeasured") {
      list_only = true;
    } else if (arg == "--strict") {
      strict = true;
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "usage: fdt_config_dump [quad.yaml] [--list-unmeasured] [--strict]\n";
      return 0;
    } else if (!arg.empty() && arg[0] == '-') {
      std::cerr << "unknown option: " << arg << "\n";
      return 2;
    } else {
      path = arg;
    }
  }

  try {
    const fdt::QuadConfig cfg = fdt::loadQuadConfig(path);
    if (list_only) {
      for (const auto& p : cfg.unmeasured) std::cout << p << "\n";
    } else {
      std::cout << fdt::summarise(cfg);
    }
    if (strict && !cfg.unmeasured.empty()) {
      std::cerr << "\nstrict: " << cfg.unmeasured.size() << " parameter(s) still unmeasured\n";
      return 1;
    }
    return 0;
  } catch (const fdt::ConfigError& e) {
    std::cerr << "config error: " << e.what() << "\n";
    return 2;
  }
}
