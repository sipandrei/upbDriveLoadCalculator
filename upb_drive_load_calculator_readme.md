# 🏎️ UPB Drive — Suspension Load & Structural Sizing Calculator

[![MATLAB](https://img.shields.io/badge/MATLAB-R2021a%2B-orange.svg)](https://www.mathworks.com/products/matlab.html)
[![Formula Student](https://img.shields.io/badge/Formula%20Student-DR07%20Suspension-blue.svg)](https://www.formulastudent.de/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)

A modular MATLAB tool developed for the **UPB Drive Formula Student** team (National University of Science and Technology POLITEHNICA Bucharest) to compute quasi-static wheel forces, decompose 3D axial loads across double-wishbone suspension members, and perform automated preliminary structural sizing for carbon-fiber wishbones, bonded metallic inserts, and critical fasteners.

---

## 📋 Table of Contents
- [Features & Modules](#-features--modules)
- [Physics & Mathematical Model](#-physics--mathematical-model)
- [Repository Structure & Inputs](#-repository-structure--inputs)
- [Generated Outputs & Visualizations](#-generated-outputs--visualizations)
- [Current Limitations & Roadmap](#-current-limitations--roadmap)
- [How to Run](#-how-to-run)

---

## 🚀 Features & Modules

The script is organized into six sequential engineering modules aligned with Formula Student technical regulations (**FS Rules 2027**):

1. **Vehicle & 3D Hardpoint Definition (`params` & `geom`)**
   * Models sprung and unsprung mass distribution, trackwidth, wheelbase, Center of Gravity ($CoG$), Center of Pressure ($CoP$), aerodynamic downforce, and roll center heights.
   * Defines 3D coordinates for all inboard/outboard pickup points (Upper/Lower Wishbones, Tie Rods, and Front Pull-Rod / Rear Push-Rod actuation links) with automatic Y-axis mirroring.
2. **Multi-Case Acceleration Sweep & Kamm Circle Filtering**
   * Evaluates a user-defined grid of longitudinal ($a_x$), lateral ($a_y$), and vertical bump ($a_z$) accelerations.
   * Automatically filters physically unrealistic combined-slip cornering/braking cases using a modified Kamm friction circle ($\sqrt{a_x^2 + a_y^2} \le 1.5\mu$).
3. **2-Stage 3D Static Equilibrium Solver (`calculateSuspensionLoads`)**
   * **Stage 1:** Solves the 6-DOF equilibrium of the upright ($\sum \mathbf{F} = 0$, $\sum \mathbf{M} = 0$) to find axial loads in the non-actuated wishbone, tie rod, and the 3D reaction force at the actuated ball joint.
   * **Stage 2:** Resolves the actuated wishbone arms and push/pull-rod forces, including secondary bending moments caused by actuation pickup offsets on CNC clevises.
4. **Carbon Fiber Tube Sizing Module (Tension, Compression & Buckling)**
   * Computes hollow circular cross-section properties ($A, I$) from outer diameter ($D_o$) and wall thickness ($t$).
   * Evaluates **Material Tensile Failure**, **Material Compressive Crushing**, and **Euler Elastic Buckling** ($K = 1.0$ pinned-pinned assumption for spherical bearings) against a target Safety Factor ($SF = 2.5$) and dynamic load multiplier ($k_{\text{dyn}}$).
5. **Adhesive Bondline Sizing Module (`L_overlap`)**
   * Calculates minimum overlap length for structural bonding between roll-wrapped CFRP tubes and 7075-T6 aluminum inserts based on adhesive shear strength (pre-configured for **3M™ Scotch-Weld™ DP460** and **EC-9323 B/A**) and bondline thickness.
6. **Critical Fastener Sizing Module (Double Shear)**
   * Determines the minimum bolt diameter required for suspension clevis joints under double shear (compliant with **FS T 2.8.3** and **T 10.1**).

---

## 🧮 Physics & Mathematical Model

### 1. Load Transfer & Contact Patch Forces
* **Vertical Loads ($F_{z,ij}$):** Combines static weight distribution, vertical bump acceleration factor ($1 + a_z/g$), aerodynamic downforce applied at $CoP$, geometric + elastic lateral load transfer (via front/rear roll center heights and roll stiffness distribution), and longitudinal weight transfer, with inside wheel-lift clamping ($F_z \ge 0$).
* **Planar Tire Forces ($F_{x,ij}, F_{y,ij}$):** Distributed along the acceleration vector up to the tire friction limit $\mu \cdot F_{z,ij}$.

### 2. Structural Verification Criteria
For each of the 24 suspension links (6 links $\times$ 4 corners), the design force is factored by the dynamic multiplier $k_{\text{dyn}}$:
$$F_{\text{design}} = k_{\text{dyn}} \cdot F_{\text{max}}$$

* **Critical Euler Buckling Load:**
  $$F_{\text{cr}} = \frac{\pi^2 \cdot E \cdot I}{(K \cdot L_{\text{tube}})^2}$$
* **Adhesive Overlap Length ($L_{\text{insert}}$):**
  $$L_{\text{insert}} = \frac{F_{\text{design}}}{\pi \cdot D_{\text{insert}} \cdot (\tau_{\text{ult}} / SF_{\text{target}})}$$
* **Minimum Bolt Diameter (Double Shear):**
  $$d_{\min} = \sqrt{\frac{4 \cdot F_{\text{design}}}{2 \cdot \pi \cdot (\tau_{\text{bolt, ult}} / SF_{\text{target}})}}$$

---

## 📊 Generated Outputs & Visualizations

Running the script generates three interactive MATLAB figures and two Command Window summary tables:
* **Figure 1 (`3D Suspension Geometry & Force Heatmap`):** Full 3D hardpoint plot with a diverging color heatmap (Blue = Compression, Red = Tension), contact patch force triads ($F_x, F_y, F_z$), and $CoG$ acceleration vectors for the worst-case load scenario.
* **Figure 2 (`4-Corner Load Envelope`):** Horizontal tornado bar charts for `FL`, `FR`, `RL`, and `RR` comparing peak tension vs. peak compression for every link and highlighting the global critical members.
* **Figure 3 (`Acceleration Load Cases`):** 2D G-G diagram and 3D acceleration space showing retained vs. filtered load cases.
* **Console Reports:** Full 24-member force envelope (`T_Envelope`), structural tube sizing table (`SizingTable`), required adhesive overlap lengths, and minimum bolt diameter.

---

## ⚠️ Current Limitations & Roadmap

* [x] Quasi-static 3D load decomposition and multi-case envelope sweep
* [x] CFRP tube buckling/tension sizing, adhesive overlap, and double-shear bolt sizing
* [ ] **Tire Model Integration:** Replace constant friction coefficient ($\mu$) with a **Pacejka Magic Formula** tire model fitted to **TTC (Tire Test Consortium)** data (including load sensitivity and self-aligning torque $M_z$).
* [ ] **Automated Tube Optimization:** Add a parametric sweep over standard carbon tube sizes ($D_o \times t$) to minimize total unsprung mass while maintaining $SF \ge 2.5$.

---

## 🛠️ How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/sipandrei/upbDriveLoadCalculator.git
   ```
2. Open the main `.m` script in **MATLAB** (no external toolboxes required).
3. Adjust vehicle parameters (`params`), hardpoint coordinates (`geom`), acceleration sweeps (`ax_range`, `ay_range`, `az_range`), or tube/adhesive properties (`sizing`) at the top of their respective sections.
4. Click **Run** to generate all console tables and 3D/2D figures.

---

*Developed as part of the Bachelor's Thesis project on Formula Student CFRP Suspension Wishbone Development — **UPB Drive Racing Team**.*