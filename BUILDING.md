
CANboat has two build systems, and they are independent — build whichever
side you need, or both.

* `make` builds the **C tools** (`analyzer`, the `*-serial` gateways, the
  converters) into `rel/<platform>/`.
* `cargo build --release` builds the **`canboat` binary and the Rust crates**
  into `target/release/`.

Both are tier 1. Neither is required to build the other.

### Compiler dependencies

For the C tools:

* GNU or compatible make tool (`make`)
* C compiler (`gcc` or `clang`, msvc not tested nor do we have build utilities)

Optional if you want to re-generate the JSON, XML and DBC files:

* `xsltproc`
* `xmllint`
* `python3`

For the Rust side, a recent stable Rust toolchain via
[rustup](https://rustup.rs/). The minimum supported version is enforced in CI;
`cargo build` will tell you if yours is too old.

### Building the Rust binary and crates

    cargo build --release

That produces `target/release/canboat`. `cargo test --workspace` runs the test
suite. The PGN database is compiled in from `database/` at build time via
`keel`, so there is nothing to load or distribute at runtime — and editing the
YAML and rebuilding is all it takes to pick up a database change.

To make the subcommands answer to the historical tool names:

    ./target/release/canboat install-shims

It only claims names that no other program already holds, so it is safe to run
in a directory where the C tools are installed.


### Building everything in [Docker](https://www.docker.com/)

```bash
make docker-build
```

or

```bash
docker build -t canboat-builder .
docker run -it --rm -v $(pwd)/:/project canboat-builder clean generated
```

### Building the binaries on Linux

The first line is for use with Debian/Ubuntu/PopOS! etc.

    sudo apt install build-essential git
    git clone https://github.com/canboat/canboat.git
    cd canboat
    make

This will probably work on Fedora:

    sudo dnf install gcc

### Building the binaries on macOS

Either install Xcode + the command line utilities in Xcode, or install `homebrew`:

    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"

Then restart a shell and do:

    brew install git python3

Then restart the shell -again- and do:

    git clone https://github.com/canboat/canboat.git
    cd canboat
    make


### Building the binaries on Microsoft Windows

It should still work with cygwin, but this is getting old and I suggest using WSL2, in which case the Linux hints above should work.

Some binaries built with cygwin are now generated again using the github continuous integration tools. These suggest that
it should build in the same way once `make` and `gcc-core` packages are installed:

    cd canboat
    make


### Building the generated code on Linux

The first line is for use with Debian/Ubuntu/PopOS! etc.

    sudo apt install build-essential git xsltproc libxml2-utils python3 python3-setuptools python3-venv
    cd canboat
    make generated

### Building the generated code on macOS

The built-in mac Python is not okay. The following will install and use homebrew instead:

    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"

Then restart a shell and do:

    brew install git python3

Then restart the shell -again- and do:

    cd canboat
    make generated


### Building the generated code on Microsoft Windows

The `generated` target is not supported.




