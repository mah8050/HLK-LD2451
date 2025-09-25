# HLK-LD2451 Radar GUI

This project is a **Python-based Graphical User Interface (GUI)** for the **HLK-LD2451 radar module**.  
It provides a user-friendly way to connect to the module, visualize detected targets in real-time, and configure its operational parameters.  

The application is built using **tkinter** for the GUI and **pyserial** for communication with the radar module over a serial port.

---

## ✨ Features

- **Serial Communication**: Easily connect and disconnect from the HLK-LD2451 radar module via a serial (COM) port.  
- **Real-time Target Visualization**: Displays a 2D radar-like view of detected targets, showing their position on an X-Y plane.  
- **Live Data Display**: Presents detailed information for up to five targets, including:
  - Position (X and Y coordinates in meters)  
  - Speed (in km/h)  
  - Distance (in meters)  
  - Angle (in degrees)  
- **Configuration Control**: Enter and exit configuration mode to change module settings.  
- **Parameter Adjustment**: Read and apply new settings for target detection and radar sensitivity, including:
  - Max Detection Distance  
  - Direction (Approach Only, Away Only, Both)  
  - Minimum Speed  
  - Target Detection Delay  
  - Trigger Count and Signal-to-Noise Ratio (SNR)  
- **Module Management**: Restart the radar module remotely.  
- **Communication Log**: Tracks all sent and received data frames, aiding in debugging and monitoring.  

---

## 📦 Requirements

- Python 3  
- Libraries:
  - `pyserial` (for serial port communication)  
  - `tkinter` (usually included with Python installation)  

---

## 🔧 Installation

```bash
# Clone the repository
git clone https://github.com/mah8050/HLK-LD2451.git
cd hlk-ld2451-gui

# Install dependencies
pip install pyserial
```

---

## ▶️ Usage

1. **Connect the Module**  
   Connect your HLK-LD2451 radar module to your computer via a USB-to-serial adapter.

2. **Run the Script**  
   Navigate to the project directory in your terminal and run:
   ```bash
   python main.py
   ```

3. **Connect to the Port**  
   - The GUI will list available COM ports.  
   - If your device is not shown, click the **Refresh** button.  
   - Select the correct COM port from the dropdown menu.  
   - Click **Connect**. The status will change from "Disconnected" to "Connected."

4. **Monitor and Configure**  
   - The radar canvas will begin to show detected targets in real-time.  
   - Use the controls on the left panel to change detection and sensitivity parameters.  
   - Remember to click **Enable Config Mode** and **Disable Config Mode** as needed.  
   - The **Communication Log** will provide feedback on all serial interactions.  

---

## 🛠️ Troubleshooting

- **Connection Errors**:  
  Ensure the correct COM port is selected and the module is properly connected and powered.  

- **No Data**:  
  If the canvas and data display remain blank after connecting, make sure you are not in configuration mode. The module only sends target data in normal mode.  

- **`ModuleNotFoundError`**:  
  A required library is missing. Make sure you installed dependencies with:  
  ```bash
  pip install pyserial
  ```

---

## 📜 License

[MIT](LICENSE)  
