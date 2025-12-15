# CADability

**CADability** is a pure **.NET class library and application** for modeling, analyzing, and interacting with 3D CAD data.  
It can be used as a backend library in your own applications or as a standalone modeling system.

---

## What is CADability?

CADability provides:

- A robust geometric and modeling kernel in **C# / .NET**
- Optional **Windows.Forms UI** for interactive use
- Support for common CAD file formats including **STEP**, **STL** and **DXF**
- Extensible data structures and modeling tools
- Parametric modeling capabilities  
- A flexible API for building custom CAD tools

This library is *standalone* and does **not** depend on external 3D modeling engines. :contentReference[oaicite:1]{index=1}

---

## Features

- Full 3D geometry and topology modeling
- Interactive model creation and editing
- Geometric analysis and queries
- Extensible UI via Windows.Forms
- Multiple CAD file export/import formats
- Modular design suitable for embedding

---

## Repository structure

The repository includes:

- **CADability** – core modeling and analysis library  
- **CADability.Forms** – optional Windows.Forms UI implementation  
- **CADability.App** – minimal executable host of the CADability.Forms UI

If you want to use CADability in your own app, typically you reference the **CADability.dll** and build your own UI around it.

---
## Documentation

An older but still useful documentation is included in the repository:

➡️ [docs/CADabilityDoc](docs/CADabilityDoc/index.html)
➡️ [Organizational classes](docs/CADabilityDoc/articles/orgclass.html)
➡️ [BRep and geometric classes](docs/CADabilityDoc/articles/database.html)
➡️ [Table of Contents](docs/CADabilityDoc/api/toc.html)

## Building

### Prerequisites

- Visual Studio 2022 or newer
- .NET 8 (or later) SDK

### Quick start

1. Clone the repository:
   ```bash
   git clone https://github.com/FriendsOfCADability/CADability.git
