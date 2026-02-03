# Standard-Header für Zeitstempel und Frame-ID (wichtig für Latenzprüfung)
std_msgs/Header header

# Identifikation
string agent_id             # Welcher Roboter/Agent sendet diesen Status?
string component_name       # Welches Bauteil (z.B. "Battery", "Arm_Joint_1", "Vacuum_Sensor")

# Datenwerte
string key                  # Der Name des Parameters (z.B. "voltage", "temperature", "load")
float64 value               # Der numerische Wert
string unit                 # Einheit (z.B. "V", "°C", "%", "N")

# Status-Klassifizierung
uint8 OK=0
uint8 WARN=1
uint8 ERROR=2
uint8 CRITICAL=3
uint8 status_code           # Aktueller Gesundheitszustand dieses Parameters

# Metadaten für das UI/Logging
string message              # Optionale Textnachricht (z.B. "Temperatur steigt schnell!")