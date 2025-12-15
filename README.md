# ShapeIt

**ShapeIt** is an open-source **3D modeler** based on the CADability geometry kernel.

The project is still under active development, but many features already work well —  
especially for **3D printing workflows**.

The main goal of ShapeIt is **simplicity of use**:  
creating and modifying solid models with as little friction as possible.

---

## Repository structure

This repository contains several closely related projects:

- **`ShapeIt`**  
  The ShapeIt 3D modeling application

- **`CADability`**  
  The geometric kernel and modeling infrastructure used by ShapeIt

- **`CADability.Forms.NET8`**  
  UI / Forms layer for .NET 8

A separate repository named **`CADability`** exists for users who rely on a *stable* version of the kernel.  
This repository (`ShapeIt`) is the **main development workspace**, where ShapeIt and ongoing CADability work live together.

---

## Branches

- **`main`** – stable snapshots  
  Suitable if you just want to build and try ShapeIt.

- **`develop`** – daily development  
  Active work happens here; things may break temporarily.

**Recommendation:**  
- Contributors → use `develop`  
- Users → use `main`

---

## Build and run

### Prerequisites

- Visual Studio 2022 (or newer)
- .NET 8 SDK
- Windows (currently required due to tooling and UI)

---

### Step 1 (optional but highly recommended): CADability debugger visualizer

Before working on ShapeIt, it is recommended to build **`CADability.sln`** once.

This will build and install a **Visual Studio Debugger Visualizer** that allows
direct visual inspection of CADability objects during debugging, e.g.:

- `Edge`
- `Face`
- `Shell`
- `Solid`
- BRep structures

Instead of interpreting raw data structures, you can *see* the geometry you are working on.
This is extremely helpful when developing or debugging the kernel.

Steps:

1. Open `CADability.sln`
2. Build the solution (Debug configuration)
3. Restart Visual Studio (required for the visualizer to become available)

---

### Step 2: Work with ShapeIt

After that, you normally work only with **`ShapeIt.sln`**.

1. Open `ShapeIt.sln`
2. Set `ShapeIt` as startup project
3. Build & run (F5)

---

## Contributing

Contributions are welcome — bug reports, fixes, refactorings, documentation, and ideas.

### Guidelines

- Check the **Issues** tab for open tasks  
  (`good first issue` and `help wanted` will be used where possible)
- For larger changes, please open an issue first to discuss the approach
- Keep pull requests focused and readable

### CADability vs ShapeIt changes

Some changes made here affect the CADability kernel.

- **General bug fixes** may later be cherry-picked into the standalone CADability repository
- **Experimental or ShapeIt-specific changes** remain here

This separation helps keep CADability stable for existing users.

---

## Project status

- ShapeIt is **usable**, but still evolving
- APIs and workflows may change
- Feedback from real usage (especially 3D printing) is very welcome

---

## License

MIT

---

## Contact / discussion

- GitHub Issues for bugs and feature requests
- Maintainer: @SOFAgh
