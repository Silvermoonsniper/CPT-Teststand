# CPT-Teststand
GUI for CPT Teststand for the cone penetration Testing with Labview. Source codes for data acquisition from distance sensor, pressure sensor, LVDT sensors, force sensor and flowmeters and Controllers. Communication between PC , Adwin system. 

The cone is attached to the force sensor with dimension of 5mm2. The pore pressure sensor and tip resistance sensor are encapsulated inside the cone. The data communication between cone and Adwin system is done with RS485 serial communciation protocol.

Force mini CPT cone to penetrate through soil samples (sand), measure soil sample deformation from LVDT sensors.

Pressure control for three air pumps to control side pressure, axial pressure and pore pressure for the CPT calibration chamber.

Motion control for the cone with switchable control methods: adaptive model predictive control (MPC) and PI controller.


# Simulation example
Motion control performance for CPT hydraulic piston with adaptive MPC. 
<br/>
Simulation setup: 

| Vibration Frequency | Vibration Amplitude | penetration depth |
| ---------| ---------| ---------|
| 5 Hz| 1 mm | 220 mm |
| 5 Hz| 2 mm | 280 mm |
| 5 Hz| 3 mm | 330 mm |

Simulation results and error analysis for whole push of VCPT testing




<p align="center">
  <img src="https://private-user-images.githubusercontent.com/89796179/295247062-209a0272-a307-4988-a770-5d3ea6e19b9b.png?jwt=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3MDQ4MTMwMDUsIm5iZiI6MTcwNDgxMjcwNSwicGF0aCI6Ii84OTc5NjE3OS8yOTUyNDcwNjItMjA5YTAyNzItYTMwNy00OTg4LWE3NzAtNWQzZWE2ZTE5YjliLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNDAxMDklMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjQwMTA5VDE1MDUwNVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTQyMTY5ZTE2ODEwYjM2ZGYwZDgwMWI5Zjg2YTEwN2NmYjhhNjA5NTJiNWFjMjAxNDE4Y2IxYjM1N2ZiODZmYTEmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0JmFjdG9yX2lkPTAma2V5X2lkPTAmcmVwb19pZD0wIn0.PMN2schoZOgy2d_8ebMulcTEB68OLC8RGH1Jl5TwR44" width="600" />
  </p>
For first vibro penetration
<p align="center">
  <img src="https://user-images.githubusercontent.com/89796179/199206693-4056337e-d6e6-48f7-b197-509d15f7e5df.png" width="600" />
  </p>



