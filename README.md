# 🔬 Kalman Filter Electron Simulation

This C++ project simulates an **electron's motion in a uniform electromagnetic field**, adds **Gaussian noise** to its position measurements, and uses a **Kalman Filter** to estimate and predict its true path.

It blends classical physics (via the Lorentz force) with modern estimation techniques from AI/ML — all implemented from scratch using clean and modular C++.

## 📌 Features

- 📐 **Physics-based Simulation**:
  - Models an electron’s trajectory using the Lorentz force:  
    **F = q(E + v × B)**
- 🔊 **Noise Addition**:
  - Adds random Gaussian noise to simulate real-world sensor errors.
- 🧠 **Kalman Filter Estimator**:
  - Estimates the true position and velocity using the noisy measurements.
- 📈 **Output Visualization**:
  - Logs data to a file (or stdout) for easy visualization using Python, MATLAB, or any plotting tool.
- ⚙️ **Header-Only Eigen Library**:
  - Uses [Eigen](https://eigen.tuxfamily.org/) for linear algebra. No need for linking external libraries.

---

## 🛠️ Build Instructions

### ✅ Prerequisites

- C++17-compatible compiler (`g++`, `clang++`, or MSVC)
- [Eigen 3](https://eigen.tuxfamily.org/) installed (header-only)

> On Linux:  
> `sudo apt install libeigen3-dev`

> On macOS with Homebrew:  
> `brew install eigen`

> On Windows:  
> Download and extract Eigen, and point the `Makefile` include path to it.

---

### 🔧 Build with Makefile

> 📁 Files:
> - `main.cpp`
> - `electron_sim.cpp`
> - `kalman.cpp`
> - `Makefile`

#### 🧱 Step-by-step:

  1. Open the `Makefile` and update the Eigen path:
   ```makefile
   CXXFLAGS = -std=c++17 -O2 -Wall -I /path/to/eigen3
  ```
  2.In terminal (Linux/macOS):
  ```bash
  make
  ./electron_si,
  ```
   On Windows with MinGW:
 ```bash
mingw32-make
electron_sim.exe
```
3.To clean build files:
   ```bash
   make clean
   ```
## 📊 Output Example
The program prints true and estimated positions of the electron. You can plot these using visualize.py:
```bash
python visualize.py
```
## 🎯 Project Motivation

This was a **personal project** born out of my passion for **particle physics** and **AI/ML**. I wanted to explore how physics-based simulations can benefit from intelligent filtering and estimation techniques.

Through this project, I learned:

- ⚡ How the **Lorentz force** governs the motion of charged particles  
- 🧠 How the **Kalman Filter** fuses model predictions with noisy observations  
- 🧩 How to design **modular simulations in C++**

---

## 📁 Project Structure
```bash

├── main.cpp # Entry point

├── electron_sim.cpp # Physics simulation (Lorentz force, noise)

├── kalman.cpp # Kalman Filter logic

├── Makefile # Cross-platform build setup

└── README.md # You're here!

```
## 🙋‍♂️ Author

**Yashasvi Kumar Tiwari**  
🇮🇳 Undergraduate CSE Student  
🌌 Aspiring Particle Physicist & ML Enthusiast  
🔗 [LinkedIn](https://www.linkedin.com/in/yashasvikumartiwari)

---

## 📫 Contributions & Feedback

I'm always open to suggestions, improvements, and feedback — especially from people working in:

- 🧪 Computational Physics  
- 📡 Signal Processing / Estimation  
- 🧠 AI for Scientific Discovery

Feel free to **fork**, **star ⭐**, or **open issues** if you find this project useful or interesting!
  
