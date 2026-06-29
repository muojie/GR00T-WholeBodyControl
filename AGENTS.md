# Repository Guidelines

## Project Structure & Module Organization

This repository hosts several whole-body control stacks. Core Python packages live in `gear_sonic/`, `decoupled_wbc/`, and `motionbricks/`. C++ deployment code and third-party runtime components are under `gear_sonic_deploy/`. Documentation sources are in `docs/`, while demo media and sample inputs are in `media/` and `sample_data/`. External vendored dependencies live in `external_dependencies/`; avoid editing them unless the change is explicitly dependency-specific. Tests currently target `decoupled_wbc/tests/`.

## Build, Test, and Development Commands

- `pip install -e gear_sonic/` installs the SONIC package for local development.
- `pip install -e "gear_sonic/[sim]"` installs MuJoCo simulation dependencies.
- `pip install -e decoupled_wbc/` installs the decoupled WBC package.
- `make run-checks` runs `isort --check`, `black --check`, and `ruff check`.
- `make format` applies `isort` and `black`.
- `pytest` runs configured tests from `decoupled_wbc/tests/`.
- `python check_environment.py` performs the pre-flight environment check recommended before PRs.

## Coding Style & Naming Conventions

Python code targets Python 3.10+. Formatting is managed by Black with a 100-character line length, while Ruff uses a 115-character line length for lint checks. Imports should follow the configured isort Black profile. Use `snake_case` for functions, modules, and variables; `PascalCase` for classes; and clear, descriptive names for scripts. Keep comments short and reserve them for non-obvious logic.

## Testing Guidelines

Use pytest for Python tests. Place new tests near the package area they exercise, and follow the configured class patterns `Test*` or `*Test`. Name test files and functions so the behavior is obvious, for example `test_mujoco_bridge_initialization.py` and `test_waits_for_first_lowcmd()`. For simulation or deployment changes, include at least a smoke command or documented manual verification if automated coverage is impractical.

## Commit & Pull Request Guidelines

The repository history mixes short imperative messages such as `feat: ...`, `fix: ...`, and Chinese documentation commits. Keep commits focused and describe the user-visible behavior. Codex-generated commit messages in this workspace must be written in Chinese. Pull requests should include a concise description, linked issue when applicable, setup or test commands run, and screenshots or logs for UI, simulation, or deployment-facing changes.

## Security & Configuration Tips

Do not commit generated MuJoCo artifacts, local model binaries, credentials, logs, or machine-specific paths. Keep large checkpoints and datasets in the documented external storage locations, and document required environment variables instead of hard-coding them.
