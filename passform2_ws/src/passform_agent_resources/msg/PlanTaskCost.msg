# PlanTaskCost.msg
# Identifikation
string request_id

# Zeitliche Kosten
builtin_interfaces/Time earliest_start
builtin_interfaces/Duration duration

# Ressourcen/Physische Kosten
float64 energy
float64 reliability  # Optional: Wert zwischen 0 und 1

# Status der Berechnung
bool success
string error_message