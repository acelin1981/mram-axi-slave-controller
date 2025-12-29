Technical Article:  
==================
Designing an AMBA-Compatible MRAM AXI Slave Controller for Modern SoCs  
https://medium.com/@ace.lin0121/designing-an-amba-compatible-mram-axi-slave-controller-for-modern-socs-3cade20bce41

MRAM AXI Slave Controller
=========================

This repository provides documentation and a reference SystemVerilog implementation
related to an MRAM-based AXI slave controller and its role in SoC-level architecture.

The materials in this repository are intended for public reference, technical
explanation, and design documentation. All files are published to provide a
verifiable, timestamped public record of the work.

RTL Structure
-------------
The design is organized into the following key components:

- `mcu_axi_master_model.sv`  
  Behavioral AXI master modeling basic MCU-style read/write traffic.

- `dma_axi_master_model.sv`  
  Behavioral AXI master modeling DMA-style memory access.

- `axi_noc_2m2s.sv`  
  AXI interconnect providing arbitration and address decoding between masters
  and slave subsystems.

- `axi_mram_slave.sv`  
  AXI slave controller interfacing with the MRAM model.

- `mram_model.sv`  
  Behavioral MRAM macro model used for functional simulation.

- `axi_ram_slave_model.sv`  
  Simple SRAM-backed AXI slave model.

- `soc_top_2m2s_noc_mram.sv`  
  Top-level SoC integration connecting all masters, interconnect, and slaves.


Repository Contents
-------------------

axi_mram_slave.sv  
A reference SystemVerilog module illustrating the structure of an AXI slave
interface for an MRAM-based memory component. This file shows example read and
write paths, address handling, and a conceptual MRAM-side request flow.  
This RTL is provided for documentation and educational purposes.

MRAM based AI SOC architecture.docx  
A document describing a high-level SoC architecture that integrates MRAM within
an AI-oriented system. It discusses design considerations, memory roles, and
how MRAM can be placed within an SoC memory hierarchy.

What is MRAM and Why It Matters in SoC Design.docx  
A technical note providing an overview of MRAM fundamentals and its relevance to
modern SoC design, including endurance, retention characteristics, and possible
usage scenarios.

README.md  
This file.

Purpose
-------

The purpose of this repository is to publish publicly accessible MRAM-related
design documentation and example RTL for reference. All materials reflect design
concepts and explanations for learning, architectural discussion, and design
traceability.

Current Status
--------------

This repository includes documentation and a conceptual RTL reference module.
It does not include verification environments, testbenches, or synthesis-ready
integrated components.

Planned Additions
-----------------

Possible future updates may include:
- Additional explanatory notes or diagrams
- Minor refinements to documentation structure

Versioning
----------

v1.0.0 – Initial public release (documentation, reference AXI MRAM Slave and example MRAM-Based SOC RTL)

Contact
-------

For questions or clarifications, please open an Issue on GitHub.
