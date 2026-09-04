<h1 align="center">AgroBot: Autonomous Precision Transplantation Platform 🌱</h1>
<h4 align="center">Zero-Relative-Velocity Kinematics, Elastoplastic Soil Mechanics, & ROS-Based Sensor Fusion</h4>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white" alt="ROS2"/>
  <img src="https://img.shields.io/badge/Python-3.10-FFD700?style=for-the-badge&logo=python&logoColor=black" alt="Python"/>
  <img src="https://img.shields.io/badge/C++-17-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white" alt="C++"/>
  <img src="https://img.shields.io/badge/Cartographer-SLAM-00FFFF?s
  <img src="https://img.shields.io/badge/SolidWorks-CAD-ED1C24?style=for-the-badge&logo=dassaultsystemes&logoColor=white" alt="CAD"/>
  <img src="https://img.shields.io/badge/Ansys-Multiphysics-8B0000?style=for-the-badge&logo=Ansys&logoColor=white" alt="Ansys"/>
  <img src="https://img.shields.io/badge/EPICS-Purdue_RSCOE-C2882B?style=for-the-badge" alt="EPICS"/>
</p>

<p align="center">
  <img src="https://via.placeholder.com/800x400/0a0a0a/00FF66?text=[AGROBOT+STRUCTURAL+CAD+RENDER+%2F+FIELD+DEPLOYMENT]" alt="AgroBot System Render" width="100%"/>
</p>

---

<details open>
  <summary><b>📑 DIRECTORY TERMINAL (TABLE OF CONTENTS)</b></summary>
  <ol>
    <li><a href="#overview">Executive Systems Overview</a></li>
    <li><a href="#datasheet">System Datasheet & Engineering Targets</a></li>
    <li><a href="#kinematics">Kinematic Modeling & Terramechanics</a></li>
    <li><a href="#hardware">Structural CAD & Mechanical Architecture</a></li>
    <li><a href="#perception">Autonomous Perception & ES-EKF Fusion</a></li>
    <li><a href="#architecture">Repository Architecture & ROS Workspace</a></li>
    <li><a href="#validation">Empirical & Simulation Validation Matrix</a></li>
    <li><a href="#institutions">Institutional Collaboration & EPICS</a></li>
    <li><a href="#citation">Academic Citation</a></li>
  </ol>
</details>

---

### <a id="overview"></a>🌐 EXECUTIVE SYSTEMS OVERVIEW

<div align="justify">
Commercial propagation of high-value cash crops—including saffron corms, greenhouse tomato seedlings, sweet bell peppers, raised-bed strawberries, and specialty wetland rice—suffers critical yield vulnerabilities driven by seasonal labor shortages, narrow biological planting windows, and high root-plug mortality from manual handling. Furthermore, heavy tractor equipment induces extensive subsoil compaction, destroying delicate capillary microstructures in raised beds.

<b>AgroBot</b> resolves these systemic bottlenecks as an autonomous, modular field-robotic platform engineered for non-destructive, sub-centimeter seedling transplantation. The architecture integrates a structurally decoupled dual-chassis configuration with a synchronized zero-relative-velocity dibbling mechanism. By translating the planting carriage rearward at a velocity strictly inverse to platform forward travel, the insertion axis executes a purely vertical soil trajectory, eliminating shear-induced root-ball tearing, bed degradation, and furrow disruption.
</div>

---

### <a id="datasheet"></a>📋 SYSTEM DATASHEET & ENGINEERING TARGETS

<div align="justify">
The operational specifications bridge theoretical kinematic synchronization, terramechanic soil-tool interaction, and edge-computed robotics:
</div>

| Subsystem | Specification | Engineering Objective |
| :--- | :--- | :--- |
| **Chassis Architecture** | Decoupled dual-frame with elastomeric damping | Isolate high-torque traction vibrations from nursery cassettes. |
| **Transplantation Engine** | Linear zero-relative-velocity vertical dibbler | Eliminate horizontal furrow shear ($\pm 0\text{ mm}$ displacement at entry). |
| **Throughput Capacity** | $68.4\text{ seedlings/hour/row}$ | Maintain continuous, non-stop autonomous bed traverse. |
| **Placement Accuracy** | Depth: $\pm 2.4\text{ mm}$ \| Pitch: $\pm 4.2\text{ mm}$ | Preserve optimal crop-specific root crown emergence and capillary access. |
| **Penetration Force ($F_z$)** | $< 118\text{ N}$ across $12\%\text{--}24\%$ soil moisture | Provide linear actuator safety factor $\eta > 2.1$ under peak loading. |
| **Perception & State** | Dual-band RTK-GPS + 2D LiDAR + RGB-D Stereo | 10-State Error-State EKF for slip-compensated row navigation. |
| **Power & Endurance** | $24\text{V}, 40\text{Ah}$ $\text{LiFePO}_4$ battery pack | Ensure $> 10.2\text{ hours}$ unassisted off-grid field runtime. |

---

### <a id="kinematics"></a>🧪 KINEMATIC MODELING & TERRAMECHANICS

#### 1. Zero-Relative-Velocity Dibbling Synchronization
<div align="justify">
To prevent furrow wall collapse and root plug tearing, the dibbler tool tip ground velocity must equal zero throughout soil penetration ($t_{\text{entry}} \le t \le t_{\text{exit}}$):
</div>

$$\mathbf{v}_{\text{net}} = \mathbf{v}_{\text{rover}} + \mathbf{v}_{\text{carriage}} = \mathbf{0} \quad \implies \quad v_{\text{carriage}}(t) = -v_{\text{rover}}(t)$$

<div align="justify">
Planar tool position $\mathbf{p}(t) = \begin{bmatrix} x(t) & z(t) \end{bmatrix}^T$ during the planting phase is governed by the synchronized parametric trajectory:
</div>

$$\begin{cases} x(t) = x_0 + \int_{0}^{t} \left(v_{\text{rover}}(\tau) - v_{\text{carriage}}(\tau)\right) d\tau \equiv x_0 \\ z(t) = z_{\text{retracted}} - \frac{h_{\text{depth}}}{2} \left[ 1 - \cos\left(\frac{2\pi t}{T_{\text{cycle}}}\right) \right] \end{cases}$$

#### 2. Coulomb-Mohr Elastoplastic Soil Mechanics
<div align="justify">
Peak vertical resistive penetration force ($F_p$) through horticultural soils ($12\%\text{--}24\%$ moisture content) is modeled using modified Bekker-Terzaghi plastic bearing capacity:
</div>

$$F_p = A_p \left( c \cdot N_c + \gamma_s z N_q + \frac{1}{2} \gamma_s b N_\gamma \right) + \mu_s \int_{0}^{z} P_h(z') \, dA_{\text{shear}}$$

<div align="justify">
Where $c$ represents soil cohesion, $\gamma_s$ is bulk soil density, $N_c, N_q, N_\gamma$ are bearing capacity factors determined by internal friction angle $\phi$, and $\mu_s$ is the steel-to-soil boundary friction coefficient.
</div>

---

### <a id="hardware"></a>⚙️ HARDWARE FABRICATION & BILL OF MATERIALS

* 🚜 **Decoupled Dual-Chassis:** Welded structural aluminum (6061-T6) divided into an upper instrumentation/cassette tier and a lower traction base, connected via high-durometer elastomeric vibration isolators to prevent structural harmonic transfer.
* 🔩 **Dibbling Core:** High-lead recirculating ball-screw driven by a high-torque closed-loop stepper, integrated onto an inverted linear rail slide for horizontal zero-velocity counter-translation.
* ⚡ **Energy & Power Distribution:** Central $24\text{V}, 40\text{Ah}$ $\text{LiFePO}_4$ battery with dedicated BMS providing galvano-isolated logic ($5\text{V}, 12\text{V}$) and traction drive rails ($24\text{V}$).

| Subsystem | Component / Specification | Functionality |
| :--- | :--- | :--- |
| **Mobility** | 4x Brushless Hub Motors + Planetary Reducers | High-torque low-velocity terramechanic row traversal |
| **Actuation** | NEMA 23 Integrated Closed-Loop Actuator + Ball Screw | Vertical dibbling and soil penetration |
| **State Estimation** | Dual-Band L1/L2 RTK-GNSS + 9-DOF IMU | Centimeter-level absolute georeferencing |
| **Vision / SLAM** | Intel RealSense D435i + RPLiDAR S2 (2D LiDAR) | Row detection, canopy clearance, Cartographer SLAM |
| **Embedded Compute** | NVIDIA Jetson Orin Nano + STM32 Motion Controller | High-level perception and real-time kinematic control |
| **Energy Unit** | $24\text{V}\ 40\text{Ah}\ \text{LiFePO}_4$ Pack with CAN BMS | Main onboard off-grid power supply |

---

### <a id="perception"></a>📡 AUTONOMOUS PERCEPTION & ES-EKF FUSION

<div align="justify">
The navigation engine resolves wheel slip, terramechanic sinkage, and crop row occlusions by implementing a <b>10-State Error-State Extended Kalman Filter (ES-EKF)</b> running at $100\text{ Hz}$.
</div>

1. **State Vector Formulation:** The system state tracking platform dynamics is defined as:
   $$\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p}^w & \delta \mathbf{v}^w & \delta \boldsymbol{\theta}^w & \delta s_{\text{slip}} \end{bmatrix}^T \in \mathbb{R}^{10}$$
2. **LiDAR & Vision Odometry Pipeline:**
   * **Planar Cartographer SLAM:** Continuously generates sub-maps for spatial boundary enforcement and headland row-end turn detection.
   * **RGB-D Vision:** Segment seedling trays, verify plug discharge, and measure terrain elevation gradients directly ahead of the dibbler assembly.
3. **Terramechanic Slip Estimation:** Wheel angular encoders are dynamically compared against RTK-derived ground velocity to continuously compute instantaneous longitudinal slip coefficients ($s_{\text{slip}}$), dynamically modifying feedforward traction torque.

---

### <a id="architecture"></a>🗄️ REPOSITORY ARCHITECTURE & ROS WORKSPACE

```text
📁 AgroBot-Autonomous-Platform/
│
├── 📁 .github/workflows/         # CI/CD: ROS 2 colcon build, flake8, gtest pipelines
├── 📁 cad_models/                 # SolidWorks & STEP assemblies for decoupled dual-chassis
├── 📁 simulations/               # Gazebo worlds, URDF/Xacro descriptions, Adams kinematics
├── 📁 data/                      # Empirical field telemetry
│   ├── penetration_logs/         # Soil force-displacement sensor profiles (12-24% moisture)
│   └── trajectory_rtk/           # High-precision ground-truth GPS traversal logs
│
├── 📁 src/                       # ROS 2 Software Stack
│   ├── agrobot_navigation/       # RTK-GNSS, Cartographer SLAM, and ES-EKF sensor fusion
│   ├── agrobot_kinematics/       # Zero-relative-velocity dibbling motion planner
│   ├── agrobot_vision/           # RealSense RGB-D nursery plug verification pipelines
│   └── agrobot_firmware/         # STM32 FreeRTOS low-level motor & actuator control
│
├── 📁 docs/                      # EPICS project reports, schematics, and wiring diagrams
├── package.xml                   # ROS 2 meta-package manifest
└── README.md                     # Main technical dossier
