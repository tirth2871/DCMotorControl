# DC Motor Speed Control using STM32 (PI Controller)

This project implements a **closed-loop DC motor speed control system** using an STM32 microcontroller, encoder feedback, and a PI controller. The system allows both predefined and user-defined RPM inputs via UART.

---

## 📌 Overview

This project demonstrates how embedded systems can be used to design a **real-time control system**. A DC motor is controlled using feedback from an encoder, and the speed is regulated using a **Proportional-Integral (PI) controller**.

---

## ⚙️ Hardware Components

- STM32F407G-DISC1 Microcontroller  
- DC Motor with Encoder  
- HD44780 LCD (I2C interface)  
- USB to UART TTL Adapter  

---

## 🔌 Key Peripherals & Configuration

- **TIM1** → Encoder mode (speed measurement)  
- **TIM7** → Control loop timing (0.01s sampling)  
- **USART2** → User input/output communication  
- **I2C** → LCD interface  

---

## 🧠 Control Strategy

- Closed-loop control using a **PI controller**  
- Error calculation:

Error = Desired RPM - Measured RPM

- Control output updates PWM signal to motor driver  
- Implemented using timer interrupts  

---

## 🔄 System Block Diagram

<img width="660" height="410" alt="block_diagram" src="https://github.com/user-attachments/assets/c8fec7eb-1f5e-4a46-976d-66da905eb96a" />


---

## 💻 Features

- Predefined RPM settings (e.g., 5, 30, 80, 130 RPM)  
- Custom RPM input via UART terminal  
- Real-time RPM display on LCD  
- Serial monitoring via terminal/plotter  
- Error tracking and correction  

---

## 📊 Results

- Achieved stable motor speed with **±2 RPM variation**  
- PI controller effectively reduced steady-state error  
- System responds to new RPM inputs within **5–10 cycles**  
- Demonstrated reliable embedded control implementation

<img width="757" height="757" alt="implementation_circuit" src="https://github.com/user-attachments/assets/f1b9004b-d435-46e6-b2db-3f4377e7f224" /> 

---

## 🚀 Learnings

- STM32 peripheral configuration (Timers, UART, I2C)  
- Encoder interfacing and RPM calculation  
- Real-time control system implementation  
- PI controller design in embedded systems  
