# STServo Controller - Production Version

A modern, robust STServo controller with web interface for precise servo motor control and monitoring of ST/SC Servos.

## 🚀 Quick Start

### Windows Users

**Option 1: PowerShell Script (Recommended)**
```powershell
.\start-servo-controller.ps1
```

**Option 2: Easy Launcher (Bypasses execution policy)**
```cmd
.\start-servo-controller-easy.bat
```

**Option 3: Manual PowerShell with bypass**
```powershell
PowerShell -ExecutionPolicy Bypass -File .\start-servo-controller.ps1
```

### Linux/macOS Users
```bash
chmod +x start-servo-controller.sh
./start-servo-controller.sh
```

## 🔧 Troubleshooting

### PowerShell Execution Policy Error

If you get this error:
```
File cannot be loaded. The file is not digitally signed.
```

**Solutions:**

1. **Use the Easy Launcher:**
   ```cmd
   .\start-servo-controller-easy.bat
   ```

2. **Run with bypass:**
   ```powershell
   PowerShell -ExecutionPolicy Bypass -File .\start-servo-controller.ps1
   ```

3. **Change execution policy (permanent fix):**
   ```powershell
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
   .\start-servo-controller.ps1
   ```

### Missing Dependencies

The scripts automatically check and install required dependencies:
- Python 3.8+
- Node.js 14+
- Flask and other Python packages
- Serve package for frontend

### Port Conflicts

If ports 5000 or 3000 are in use, the backend will automatically find an available port.

## 🌐 Access

Once started, access the application at:
- **Frontend**: http://localhost:3000
- **Backend API**: http://localhost:5000

## 📁 Project Structure

```
STServo-Production/
├── backend/
│   └── app.py              # Flask API server
├── frontend-build/         # React frontend (built)
├── STservo_sdk/           # STServo SDK
├── requirements.txt       # Python dependencies
├── start-servo-controller.ps1    # Windows PowerShell script
├── start-servo-controller.sh     # Linux/macOS shell script
├── start-servo-controller-easy.bat # Easy launcher for Windows
└── README.md              # This file
```

## 🎮 Features

- **Real-time Control**: Direct servo position and speed control
- **Telemetry**: Live monitoring of servo status
- **Cross-platform**: Works on Windows, Linux, and macOS
- **Auto-setup**: Automatic dependency installation and environment setup
- **Two-terminal**: Separate windows for backend and frontend logs

## 🛑 Stopping the Servers

- **Windows**: Close the terminal windows or run:
  ```cmd
  taskkill /f /im python.exe && taskkill /f /im node.exe
  ```
- **Linux/macOS**: Close the terminal windows or run:
  ```bash
  pkill -f 'python.*backend/app.py' && pkill -f 'npx serve.*frontend-build'
  ```

## 📝 Requirements

- **Python**: 3.8 or higher
- **Node.js**: 14 or higher
- **Operating System**: Windows 10+, macOS 10.14+, or Linux

## 🔄 Updates

The scripts automatically check for existing dependencies and only install what's missing, making subsequent runs faster.

## 📊 Build Information

- **Frontend**: React 18.2.0 with Tailwind CSS
- **Backend**: Flask 2.3.3 with CORS support
- **Build Date**: 30-08-2025 
-- **Version**: 1.0.0 Production
-
