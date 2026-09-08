# Instructions for agents

These instructions apply to the entire repository.

## Supported machines

This is firmware for four desktop CNC mills:

- Carvera (C1)
- Carvera Air (CA1)
- Makera Z1
- Makera Z1 Pro

All four are Cartesian three-axis machines with an optional rotary A axis. The
Carvera has an automatic tool changer (ATC). The Carvera Air does not have a
factory ATC, although some machines have a user-installed ATC modification. The
Z1 and Z1 Pro do not have an ATC.

The built-in C1 configuration is `src/config.default`. The built-in CA1
configuration is `src/config2.default`. The built-in Z1 and Z1 Pro
configurations are `src/config_z1.default` and `src/config_z1pro.default`.
Machine-specific settings supplement the built-in configuration, so the
effective configuration is not necessarily the same as the corresponding file
in the repository.

This code descends from general-purpose Smoothieware and still contains code
for machines and kinematics that Carvera does not use, including CoreXY and
SCARA. Do not assume that inherited capabilities are supported here. Do not
work on unrelated generic-machine code unless the task explicitly requires it.
Do not infer that different machine models share a feature merely because
related code or configuration entries exist.

## Working in this repository

The codebase is messy and contains legacy and unused code. The presence of a
class, option, or code path does not prove that current firmware uses it. Trace
callers, build rules, configuration, and runtime use before relying on it.

Validate every assumption about the code, the target machine, and its
configuration that affects the work. Use the source and build configuration for
code facts. For hardware facts, start with the machine description above and
ask the user about the actual machine and its SD-card overlay. If the available
evidence does not settle an assumption, ask the user. Do not guess or continue
work that depends on an unverified assumption.

Keep changes narrowly tied to the request. Do not clean up nearby legacy code,
reformat unrelated files, or make a generic Smoothieware improvement as part of
a Carvera change.

`docs/commands.json` is the structured reference for G-codes, M-codes, and
console commands. When you add or change a command, update that file so the
command ID, parameters, and defaults match the firmware. CI requires new
command IDs in a pull request to appear there. The text must be consise in this
file as it's used for "tooltip" like guidance, it's not a exhaustive guide.

This is an embedded project. Flash, AHB SRAM, and regular SRAM are limited
resources. Treat increases in any of them as a cost that must be understood and
justified. Consider resource use when choosing an implementation.

## Validation

Build Carvera and Carvera Air firmware with the Make-based build wrapper:

```sh
./build/build.sh --clean
```

Build Makera Z1 and Makera Z1 Pro firmware with:

```sh
./build/build.sh --clean MACHINE=z1
```

This repository does not currently have a functional test suite. A successful
build or static check does not show that the machine still behaves correctly.

Before submitting a change that affects machine behavior, test it on a real
supported machine that has the affected feature. Record the information
relevant to the testing in the pull request. The maintainers do not have the
resources to perform this validation for contributors, and an unvalidated
behavior change is not ready for review.

## Final review and pull requests

Before presenting or submitting a change:

* Read and understand every changed line. The person submitting the pull
  request must be able to explain and take responsibility for all of it.
  This applies to both you and the user.
* Review the complete diff and remove over-documentation, LLM-speak,
  invented terminology, and process artifacts such as design documents,
  execution plans, prompts, and progress notes. Remove the generated
  binaries and unrelated formatting or cleanup changes.
* Make sure plain, ordinary English is used in names, comments, documentation
  and commit messages. Comments must explain the code as it exists now.
  Do not leave comments that narrate what the code used to do, what was removed,
  or how the change was produced.

Submitting a PR:

* In the PR body, describe the final behavior, motivation, configuration,
  compatibility impact, and validation. Do not narrate the development process
  or commit history.
* Do not mention internal workflow details that are not included in (or not
  relevant to) the final change, such as temporary files or branches or
  development history.
* Do not use private or invented terminology that the maintainers might be
  not familiar with, or that is not in common use in the project.
* Never complete first-person attestations or contributor declarations on behalf
  of the user.
* Keep the required AI disclosure factual and brief. It should identify the tool
  and the kind of assistance, not recount the development process.
* Report validation honestly. Do not imply that a build is a functional test.
