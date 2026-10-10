"""Compare native replay captures without updating reviewed fixture expectations."""

import argparse
import hashlib
import json
import pathlib
import re
import sys


FULL_MATRIX = tuple(
    f"{os_name}-{architecture}-{configuration}"
    for os_name in ("windows", "linux")
    for architecture in ("x64", "arm64")
    for configuration in ("Release", "ReleaseLean")
)
FIXTURE_NAMES = (
    "both-lifecycle-v1", "mixed-lifecycle-v1", "three-d-caches-v1",
    "three-d-lifecycle-v1", "three-d-surface-contacts-v1", "two-d-lifecycle-v1",
)
SOURCE_DEPENDENCIES = {"FixedMathSharp", "SwiftCollections", "GridForge", "Chronicler"}
DEPENDENCIES = {
    "Gravitas", "FixedMathSharp", "FixedMathSharp.Chronicler", "SwiftCollections",
    "SwiftCollections.FixedMathSharp", "GridForge", "Chronicler",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def canonical(value):
    # JSON preserves bool/int/float distinctions that Python container equality loses.
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def read_json(path):
    def object_pairs(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, f"Duplicate JSON property '{key}'")
            result[key] = value
        return result

    def reject_constant(value):
        raise ValueError(f"Nonfinite JSON number '{value}'")

    try:
        value = json.loads(path.read_text(encoding="utf-8-sig"),
                           object_pairs_hook=object_pairs, parse_constant=reject_constant)
        canonical(value)
        return value
    except (OSError, ValueError) as error:
        raise ValueError(f"Invalid or missing JSON '{path}': {error}") from error


def validate_frames(frames, count, context):
    require(isinstance(frames, list) and len(frames) == count,
            f"{context}: incomplete or extra frames; expected exactly {count}")
    for index, frame in enumerate(frames):
        require(isinstance(frame, dict), f"{context}: frame {index + 1} must be an object")
        for field in ("FrameCount", "LateToken"):
            require(type(frame.get(field)) is int and frame[field] == index + 1,
                    f"{context}: frame {index + 1} {field} must be {index + 1}")


def read_fixtures(directory):
    require(directory.is_dir(), f"Missing fixtures directory '{directory}'")
    expected_files = {name + ".json" for name in FIXTURE_NAMES}
    actual_files = {path.name for path in directory.iterdir()}
    require(actual_files == expected_files,
            f"Fixtures file set mismatch: missing={sorted(expected_files - actual_files)}, "
            f"unexpected={sorted(actual_files - expected_files)}")
    fixtures, identities = {}, {}
    for name in FIXTURE_NAMES:
        path = directory / (name + ".json")
        fixture = read_json(path)
        require(isinstance(fixture, dict), f"Fixture '{name}' must be an object")
        require(fixture.get("Name") == name, f"Fixture '{name}' Name mismatch")
        require(type(fixture.get("Version")) is int and fixture["Version"] == 1,
                f"Fixture '{name}' Version must be 1")
        require(fixture.get("RuntimeMode") in ("TwoD", "ThreeD", "Both", "Mixed"),
                f"Fixture '{name}' RuntimeMode is invalid")
        count = fixture.get("FrameCount")
        require(type(count) is int and count > 0, f"Fixture '{name}' FrameCount must be positive")
        validate_frames(fixture.get("Expected"), count, f"Fixture '{name}' Expected")
        commands = fixture.get("Commands")
        require(isinstance(commands, list), f"Fixture '{name}' Commands must be an array")
        previous = -1
        for command in commands:
            require(isinstance(command, dict) and type(command.get("Frame")) is int,
                    f"Fixture '{name}' command Frame must be an integer")
            require(previous <= command["Frame"] < count and command["Frame"] >= 0,
                    f"Fixture '{name}' Commands must be ordered within FrameCount")
            previous = command["Frame"]
        fixtures[name] = fixture
        identities[name] = {"Version": fixture["Version"],
                            "Sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return fixtures, identities


def validate_provenance(value, lane, fixture_identities):
    context = f"Lane '{lane}' provenance"
    require(isinstance(value, dict), f"{context} must be an object")
    require(type(value.get("Version")) is int and value["Version"] == 1,
            f"{context} Version must be 1")
    os_name, architecture, configuration = lane.split("-")
    expected = {"OS": os_name.title(), "OSArchitecture": architecture.title(),
                "ProcessArchitecture": architecture.title(), "Configuration": configuration}
    for field, wanted in expected.items():
        require(value.get(field) == wanted,
                f"{context} {field} must be '{wanted}' for native execution")
    local_stack = value.get("UseLocalLsfStack")
    require(type(local_stack) is bool, f"{context} UseLocalLsfStack must be a boolean")
    for field in ("OSDescription", "Framework", "RuntimeVersion", "SDKVersion"):
        require(isinstance(value.get(field), str) and value[field].strip(),
                f"{context} {field} must be a nonempty string")
    for field, pattern in (("Framework", r"\.NET 8\.0\.\d+"),
                           ("RuntimeVersion", r"8\.0\.\d+"),
                           ("SDKVersion", r"10\.0\.\d+"),
                           ("SourceRevision", r"[0-9a-f]{40}")):
        require(isinstance(value.get(field), str) and re.fullmatch(pattern, value[field]),
                f"{context} {field} has an invalid value")
    sources = value.get("SourceDependencies")
    expected_sources = SOURCE_DEPENDENCIES if local_stack else set()
    require(isinstance(sources, dict) and set(sources) == expected_sources,
            f"{context} SourceDependencies must identify all four repositories in source mode "
            "and be empty in package mode")
    for name, revision in sources.items():
        require(isinstance(revision, str) and re.fullmatch(r"[0-9a-f]{40}", revision),
                f"{context} SourceDependencies '{name}' must be a full lowercase commit")
    dependencies = value.get("Dependencies")
    require(isinstance(dependencies, list), f"{context} Dependencies must be an array")
    versions = {}
    for dependency in dependencies:
        require(isinstance(dependency, dict), f"{context} Dependencies entry must be an object")
        name = dependency.get("Name")
        require(isinstance(name, str) and name in DEPENDENCIES and name not in versions,
                f"{context} Dependencies has an unknown or duplicate Name")
        for field in ("Version", "InformationalVersion", "Sha256"):
            require(isinstance(dependency.get(field), str) and dependency[field].strip(),
                    f"{context} Dependencies '{name}' {field} must be nonempty")
        require(re.fullmatch(r"\d+\.\d+\.\d+\.\d+", dependency["Version"]),
                f"{context} Dependencies '{name}' Version must be an assembly version")
        require(re.fullmatch(r"[0-9a-f]{64}", dependency["Sha256"]),
                f"{context} Dependencies '{name}' Sha256 must be 64 lowercase hexadecimal characters")
        # Source revisions identify development builds even when build metadata varies.
        # Released packages must also agree on their loaded release identity.
        versions[name] = dependency["Version"] if local_stack else {
            "Version": dependency["Version"], "InformationalVersion": dependency["InformationalVersion"]
        }
    require(set(versions) == DEPENDENCIES, f"{context} Dependencies must identify all seven assemblies")
    require(canonical(value.get("Fixtures")) == canonical(fixture_identities),
            f"{context} Fixtures versions or manifest SHA256 fingerprints do not match source fixtures")
    return {"UseLocalLsfStack": local_stack, "SourceRevision": value["SourceRevision"],
            "SourceDependencies": sources, "Dependencies": versions}


def divergence(fixture, index, lane, expected, actual, comparison):
    commands = [command for command in fixture["Commands"] if command["Frame"] <= index]
    return (
        f"Fixture '{fixture['Name']}' version {fixture['Version']}, mode {fixture['RuntimeMode']}, "
        f"lane '{lane}': first divergent frame {index + 1} (index {index}); {comparison}.\n"
        "Preceding commands (including this frame):\n" + canonical(commands)
        + "\nExpected raw state and ordered events/queries:\n" + canonical(expected)
        + "\nActual raw state and ordered events/queries:\n" + canonical(actual)
    )


def compare_captures(captures, fixtures, lanes=None):
    captures, fixtures = pathlib.Path(captures), pathlib.Path(fixtures)
    lanes = FULL_MATRIX if lanes is None else tuple(lanes)
    require(len(lanes) >= 2, "At least two native lanes are required for cross-lane comparison")
    require(len(lanes) == len(set(lanes)), "Selected lanes must be unique")
    require(all(lane in FULL_MATRIX for lane in lanes), "Selected lanes must belong to the full native matrix")
    manifests, fixture_identities = read_fixtures(fixtures)
    expected_files = {name + ".actual.json" for name in FIXTURE_NAMES} | {"provenance.json"}
    traces, baseline_identity = {}, None
    for lane in lanes:
        directory = captures / lane
        require(directory.is_dir(), f"Missing required lane '{lane}' at '{directory}'")
        actual_files = {path.name for path in directory.iterdir()}
        require(actual_files == expected_files,
                f"Lane '{lane}' capture file set mismatch: missing={sorted(expected_files - actual_files)}, "
                f"unexpected={sorted(actual_files - expected_files)}")
        identity = validate_provenance(read_json(directory / "provenance.json"), lane, fixture_identities)
        if baseline_identity is None:
            baseline_identity = identity
        else:
            for field in baseline_identity:
                require(canonical(identity[field]) == canonical(baseline_identity[field]),
                        f"Lane '{lane}' provenance {field} differs from baseline lane '{lanes[0]}'")
        traces[lane] = {}
        for name, fixture in manifests.items():
            frames = read_json(directory / (name + ".actual.json"))
            validate_frames(frames, fixture["FrameCount"], f"Lane '{lane}' fixture '{name}'")
            traces[lane][name] = frames

    for name, fixture in manifests.items():
        for index, expected in enumerate(fixture["Expected"]):
            baseline = traces[lanes[0]][name][index]
            for lane in lanes[1:]:
                actual = traces[lane][name][index]
                if canonical(baseline) != canonical(actual):
                    raise ValueError(divergence(fixture, index, lane, baseline, actual,
                                                f"direct comparison with baseline lane '{lanes[0]}'"))
            for lane in lanes:
                actual = traces[lane][name][index]
                if canonical(expected) != canonical(actual):
                    raise ValueError(divergence(fixture, index, lane, expected, actual,
                                                "shared expectation comparison"))
    unexecuted = [lane for lane in FULL_MATRIX if lane not in lanes]
    scope = "Selected native matrix passed" if unexecuted else "Full native matrix passed"
    return (f"{scope}: {len(lanes)} lanes, {len(manifests)} fixtures; shared expectations and "
            f"direct cross-lane captures agree.\nExecuted lanes: {', '.join(lanes)}\n"
            f"Unexecuted full-matrix lanes: {', '.join(unexecuted) if unexecuted else 'none'}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--captures", required=True, type=pathlib.Path)
    parser.add_argument("--fixtures", required=True, type=pathlib.Path)
    parser.add_argument("--lanes", nargs="+", choices=FULL_MATRIX,
                        help="Explicit submatrix of at least two lanes; default requires all eight")
    args = parser.parse_args(argv)
    try:
        print(compare_captures(args.captures, args.fixtures, args.lanes))
    except (OSError, ValueError) as error:
        print(f"Replay capture comparison failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
