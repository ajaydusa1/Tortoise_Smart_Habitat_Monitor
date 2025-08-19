# Tortoise Smart Habitat Monitor 🐢  

An **STM32-based automation system** for maintaining a healthy environment for star tortoises.  
It controls **UV/heat lamps**, **ambient LED lighting (PWM with fade effect)**, and displays **temperature, humidity, and real-time clock (RTC)** values on an LCD.  

---

## ✨ Features
- DS3231 RTC for accurate timekeeping  
- AHT20 Temp-Humidity sensor monitoring  
- 8x2 LCD display with auto screen switching (Temp/Humidity ↔ Time/Status)  
- Relay control for UV/heat lamps  
- Ambient LED strip with smooth PWM fading effect  
- Fully configurable timings via code variables  

---

## 🛠️ Hardware
- STM32 Microcontroller  
- DS3231 RTC Module  
- AHT20 Temperature & Humidity Sensor  
- 8x2 LCD (HD44780 driver)  
- Relay Module (for UV lamp & heater)  
- LED Strip (PWM controlled)  

---

## 📷 Demo
(Add pictures or wiring diagram here)  

---

## 🚀 How It Works
1. The system reads **temp & humidity** periodically.  
2. Displays readings and current **time** alternately on LCD.  
3. Controls **relays** to switch UV/heat lamps.  
4. Provides **PWM ambient lighting** with sunrise/sunset fading effect.  

---

## 📄 License
This project is licensed under the MIT License.  

---
