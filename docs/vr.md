# VR Application Installation

You can set up the VR app in **one** of two ways:

1. **Install the pre-built APK** directly onto your Quest headset with SideQuest.  Check [`installation instructions`](https://github.com/ARCLab-MIT/BeaVR-app/tree/main/BeaVR-Unity)

Once the app is on the headset, enter the IP address of the robot server. The robot server **and** the Quest must be on the same network.

---

## User Interface – Single Robot Arm + Hand

Because the robot hand is right-handed, all mode-switch gestures are performed with the **left** hand, leaving the right hand free for key-point streaming.

| Left-Hand Pinch | Mode              | Stream Border Color |
| --------------- | ----------------- | ------------------- |
| Index           | **Hand Only**     | 🟢 Green            |
| Middle          | **Arm + Hand**    | 🔵 Blue             |
| Ring            | **Pause**         | 🔴 Red              |
| Pinky           | **Resolution Select** | ⚫ Black        |

> **Tip:** The app runs continuously. If the robot arm connects successfully, the border will turn (or stay) **green**. A non-green border on first interaction indicates no active connection.

---

## Getting Started

1. Launch the app on your Quest. A blank screen with a red border and **Menu** button appears.  
2. Enable hand tracking (Quest settings).  
3. Tap **Menu ▸ Change IP**, enter the robot-server IP, then go back and tap **Stream**.  
4. When the border turns **green**, the headset is streaming key-points to the server.

The APK files are available **[here](/VR/APK/)**.  
After setup, continue to the **[teleoperation guide](/docs/teleop_data_collect.md)**.

> **Note:** Make sure the same IP is set in the server-side config: [`configs/network.yaml`](/configs/network.yaml).
