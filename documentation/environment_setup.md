# Simulation Environment Setup & Framework Compatibility Guide

This document provides a comprehensive technical guide for setting up the vehicular and network simulation toolchains for this project.

---

## 1. Current Host Environment

- **Operating System**: Windows 11 / Windows 10 x64 (Build 10.0.17763 / Windows 11 AMD64)
- **Python**: 3.12.10 (Path: `C:\Users\ASUS\AppData\Local\Programs\Python\Python312\python.exe`)
- **Eclipse SUMO**: Version 1.28.0 (Installed at `C:\Program Files (x86)\SUMO\`)
- **SUMO_HOME**: `C:\Program Files (x86)\SUMO\`
- **TraCI**: Python TraCI package verified and operational.
- **Available Disk Space**: 209+ GB free on Drive `C:\`.

---

## 2. Framework Compatibility Matrix

Vehicular network simulation requires tight coupling between three core frameworks:
1. **Network Simulator**: OMNeT++
2. **Network Protocol Library**: INET Framework
3. **Vehicular Mobility Bridge**: Veins (Vehicles in Network Simulation) / TraCI
4. **Microscopic Road Traffic Simulator**: SUMO

The following compatibility combinations are established for academic and industry VANET research:

| Component | Modern Recommended Stack | Classic Legacy Stack |
|---|---|---|
| **OMNeT++** | **6.0.3** (Windows x86_64) | 5.7.1 (Windows x86_64) |
| **INET Framework** | **INET 4.5.2** | INET 4.2.8 / 4.3.2 |
| **SUMO** | **1.28.0** (Already Installed) | 1.12.0 – 1.18.0 |
| **Routing Protocol** | INET Native AODV (`inet.routing.aodv`) | INET AODV |

> **Recommended Target**: **OMNeT++ 6.0.3** + **INET 4.5.2**. This pairing includes full official support for Windows 10/11 64-bit, native Clang toolchains, integrated IDE, and a fully tested, RFC-compliant AODV routing implementation out-of-the-box.

---

## 3. Step-by-Step Installation Guide for OMNeT++ 6.0.3

OMNeT++ on Windows comes packaged as an all-in-one archive containing a dedicated MSYS2/MinGW toolchain, compilers, Qt GUI libraries, and the Eclipse-based IDE.

### Step 3.1: Download Archive
1. Visit the official OMNeT++ download repository:
   `https://omnetpp.org/download/old` or `https://github.com/omnetpp/omnetpp/releases/download/omnetpp-6.0.3/omnetpp-6.0.3-windows-x86_64.zip`
2. Download `omnetpp-6.0.3-windows-x86_64.zip` (approximately 1.1 GB).

### Step 3.2: Extraction
1. Extract the downloaded archive directly into the root of `C:\`:
   Target folder: `C:\omnetpp-6.0.3`
   *(Important: Do NOT extract into paths containing spaces or special characters, such as `Program Files`)*.

### Step 3.3: Configuration and Compilation
1. Navigate to `C:\omnetpp-6.0.3` and double-click `mingwenv.cmd`.
   This opens the embedded MSYS2 development terminal with all environment variables configured.
2. Inside the MSYS2 shell, run the configure script:
   ```bash
   ./configure
   ```
3. Once configuration completes successfully, compile OMNeT++:
   ```bash
   make -j4
   ```
4. Test the installation by running a sample simulation (e.g. `aloha`):
   ```bash
   cd samples/aloha
   ./aloha
   ```
5. Launch the IDE:
   ```bash
   omnetpp
   ```

---

## 4. INET Framework Installation & AODV Verification

INET provides standard Internet protocols, wireless channel modeling, and reactive ad-hoc routing protocols (AODV).

### Step 4.1: Obtain INET 4.5.2
1. Download INET 4.5.2 from GitHub:
   `https://github.com/inet-framework/inet/releases/download/v4.5.2/inet-4.5.2-src.tgz`
2. Extract the archive into your OMNeT++ workspace directory or alongside OMNeT++:
   e.g. `C:\omnetpp-6.0.3\samples\inet4.5`

### Step 4.2: Import and Build in OMNeT++ IDE
1. In OMNeT++ IDE, click `File -> Import... -> General -> Existing Projects into Workspace`.
2. Browse to the extracted `inet4.5` folder.
3. Select `inet` project and check "Search for nested projects".
4. Click `Finish`.
5. Right-click the `inet` project in Project Explorer -> `Build Project`.

### Step 4.3: Verification of Native INET AODV Implementation
INET contains a complete, RFC 3561-compliant implementation of AODV located at:
`inet/src/inet/routing/aodv/`
- **NED Module**: `inet.routing.aodv.AodvRouter` / `Aodv`
- **C++ Headers & Source**:
  - `Aodv.h` / `Aodv.cc`: Main protocol engine handling routing tables, timers, and route discovery.
  - `AodvControlPackets.msg`: Message definitions for:
    - `AodvRreq` (Route Request)
    - `AodvRrep` (Route Reply)
    - `AodvRerr` (Route Error)
    - `AodvRrepAck` (Route Reply Acknowledgment)
- **Built-in AODV Test Showcase**:
  Verify the AODV routing protocol by running the showcase simulation:
  `inet/showcases/wireless/aodv/omnetpp.ini`
  This verifies that mobile wireless nodes discover routes dynamically, handle intermediate link breaks, and route packets end-to-end without static configuration.
