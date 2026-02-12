# AMBA-Compatible MRAM AXI Slave Controller (SystemVerilog)

## Technical Article:
Designing an AMBA-Compatible MRAM AXI Slave Controller for Modern SoCs  
https://medium.com/@ace.lin0121/designing-an-amba-compatible-mram-axi-slave-controller-for-modern-socs-3cade20bce41

---

## Overview
This repository contains a SystemVerilog implementation of an AMBA AXI-compatible MRAM slave controller and related SoC-level models used for functional validation and power/energy analysis in simulation.

The project includes:
- AXI NoC (2 masters / 3 slaves) integration example
- MRAM controller + MRAM behavioral model
- DDR4 / SRAM / generic RAM slave models
- System-level memory power monitor
- Testbench for frame-based workload simulation (plusargs configurable)

---

## Repository Structure
```text
.
├── src/                 # RTL modules (SystemVerilog)
│   ├── design_preamble.sv
│   ├── axi_noc_2m3s.sv
│   ├── axi_mram_slave_ctrl.sv
│   ├── mram_model.sv
│   ├── axi_ddr4_slave_model.sv
│   ├── axi_sram_slave_model.sv
│   ├── axi_ram_slave_model.sv
│   ├── soc_mem_power_monitor.sv
│   └── npu_trace_axi_master_no_reuse.sv
├── tb/                  # Testbench
│   └── testbench_acelin1981.sv
├── rtl.f                # Icarus Verilog filelist
├── run.bat              # One-click compile + run on Windows
└── README.md
```

---

## Quick Start (Windows + Icarus Verilog)

### Prerequisites
Install **Icarus Verilog** (`iverilog` and `vvp` available in PATH).

Verify in PowerShell:
```powershell
iverilog -V
vvp -V
```

### Build & Run
In the project root directory:
```powershell
.\run.bat 100
```

---

## Simulation Parameters (Plusargs)
The testbench accepts runtime plusargs via `vvp`.

`WEIGHT_MRAM_PCT` options:
**0, 20, 40, 60, 80, 100**

Examples:
```powershell
.\run.bat 0
.\run.bat 60
.\run.bat 100
```

Default arguments used in `run.bat`:
- `+SCALE=256`
- `+WEIGHT_MODE=C`
- `+WEIGHT_MRAM_PCT=<0|20|40|60|80|100>`

---

## Notes
- `design_preamble.sv` is included by RTL modules using:
  ```sv
  `include "design_preamble.sv"
  ```
  Therefore the compile command uses `+incdir+./src`.

- If you add new RTL files under `src/`, remember to update `rtl.f`.

---

## License
Copyright (c) acelin1981  
Academic/Research use is permitted (non-commercial) with attribution.  
Commercial use requires prior written permission from the copyright holder.
