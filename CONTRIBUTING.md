# Contributing

Thanks for helping improve Gravitas. Determinism and physics correctness come
first; focused changes with explicit ownership and verification are easiest to
review.

For substantial API, architecture, serialization, or physics changes, open an
issue or discussion before implementation so the intended contract is clear.
Focused fixes can proceed directly through a pull request.

## Development Setup

[`global.json`](global.json) selects the .NET SDK used for solution tooling.
Keep the .NET 10 runtime installed for the `net10.0` tests and benchmarks.

```bash
dotnet restore Gravitas.slnx
dotnet build Gravitas.slnx --configuration Release
dotnet test Gravitas.slnx --configuration Release
```

Run the Lean configuration when a change touches dependencies, serialization,
MemoryPack conditionals, or shared runtime APIs:

```bash
dotnet build Gravitas.slnx --configuration ReleaseLean
dotnet test Gravitas.slnx --configuration ReleaseLean
```

Build the documentation site after a Release build when changing XML docs,
`docs/api`, or API navigation:

```bash
dotnet tool restore
dotnet tool run docfx docs/api/docfx.json --warningsAsErrors
```

See [AGENTS.md](AGENTS.md) for the full architecture, determinism, testing,
serialization, benchmark, and local LSF stack guidance.

## Coordinated source identities

Use `-p:UseLocalLsfStack=true -p:DisableTransitiveProjectReferences=true` for
coordinated sibling builds and serialize builds with `-m:1`.
`Directory.Build.props` owns the shared runtime version selection and local
reference identities. Gravitas uses `0.0.0-grv-local-lsf`; siblings use their
corresponding evergreen labels. Assembly/file versions are `0.0.0.0`, even when
GitVersion values are present. These identities do not change for releases.
Package mode remains the release-validation path.

## Pull Request Process

1. Keep the change focused and preserve deterministic behavior, stable ordering,
   and engine-agnostic runtime boundaries.
2. Add or update tests for behavior changes and keep the README, guides, XML
   documentation, and benchmarks aligned with the affected public contract.
3. Run the `Release` build and tests. Also run `ReleaseLean` when changing
   packages, serialization, MemoryPack conditionals, or shared runtime APIs.
4. Do not hand-edit package versions for normal releases; repository workflows
   derive versions through GitVersion.
5. Exclude generated build, test, coverage, package, and benchmark artifacts
   from the pull request.

## Documentation

- Keep the root README short, welcoming, and focused on why someone would use
  Gravitas.
- Put task-oriented guidance and cross-cutting behavior in `docs/api/guides`;
  keep links between guide pages relative and include their `.md` extensions.
- Keep public XML documentation factual and useful because it feeds the DocFX
  API reference.
- Update the matching guide whenever lifecycle, dimensional behavior, collision,
  queries, serialization, replay, or diagnostics change.

Please follow the [Code of Conduct](CODE_OF_CONDUCT.md) in all project
interactions. See [Support](.github/SUPPORT.md) for help and
[Security](SECURITY.md) for private vulnerability reports.

## Documentation ownership and cleanup

Conceptual guides publish alongside the API reference from `docs/api/guides`.
The repository Markdown is the source of truth; there is no wiki synchronization
workflow. Keep guides evergreen and preserve completed design and validation
records under `docs/feature-work/done` as history.

Give each namespace-level type or delegate its own file. Public/internal nested
types use companion files while preserving containing types, generic
constraints, attributes and CLR identities. Private nested helpers may stay with
their owner. Authored C# files under `src` carry the current filename and MIT
license header. Test and benchmark files under `tests` have no license headers.

Run cleanup explicitly before review using the SDK from `global.json` and
Node.js/npm for Prettier:

```powershell
$env:TargetFramework='net10.0'
dotnet format Gravitas.slnx --severity error
Remove-Item Env:TargetFramework
npx --yes prettier@3.9.5 --write .
```

Select one target framework for formatting to avoid competing fixes in shared
multi-target files. Keep imports needed by other targets inside their matching
conditional block, and validate both target frameworks after cleanup. Clear the
framework selection before normal multi-target builds. If workspace loading
reports unmatched project/metadata references, stop semantic fixes and
clean/rebuild before retrying. Use the existing local-stack environment flags
when coordinated-source formatting is needed.

The .NET formatter follows `.editorconfig`; it does not reproduce a Visual
Studio cleanup profile. Prettier is pinned in the command and may download on
first use. `.prettierignore` excludes generated output and reviewed replay
fixtures whose exact file hashes identify conformance inputs. No commit hook
rewrites or stages files.

Check without rewriting files:

```powershell
$env:TargetFramework='net10.0'
dotnet format Gravitas.slnx --severity error --verify-no-changes
Remove-Item Env:TargetFramework
npx --yes prettier@3.9.5 --check .
```
