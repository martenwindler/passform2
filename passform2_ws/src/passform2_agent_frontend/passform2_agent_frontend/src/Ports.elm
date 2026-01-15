port module Ports exposing (..)

import Json.Decode as Decode
import Json.Encode as Encode

-- --- OUTGOING (Elm -> JavaScript) ---
-- Diese Ports senden Befehle von Elm an dein JavaScript (index.ts)

-- Verbindung zum Backend (Socket.io) herstellen
port connectToBackend : String -> Cmd msg

-- Zwischen Simulation und Hardware Modus umschalten
port setMode : String -> Cmd msg

-- Der 3D-Szene sagen, ob sie 2D oder 3D anzeigen soll
port setViewMode : Bool -> Cmd msg

-- Die aktuelle Gitter-Konfiguration als JSON-Download exportieren
port exportConfig : String -> Cmd msg

-- Den Datei-Browser öffnen, um eine Konfiguration zu importieren
port importConfigTrigger : () -> Cmd msg

-- Das aktuelle Layout im LocalStorage des Browsers sichern
port saveToLocalStorage : String -> Cmd msg

-- Die Pfadplanung im Python-Backend triggern (sendet JSON)
port triggerPlanning : Encode.Value -> Cmd msg


-- --- INCOMING (JavaScript -> Elm) ---
-- Diese Ports empfangen Daten von JavaScript/Backend und leiten sie an Elm weiter

-- Empfängt die aktuelle Liste der Agenten (z.B. bei Hardware-Updates)
port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt den berechneten Pfad vom Backend (Event: "path_complete")
port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt den Verbindungsstatus (True = Online, False = Offline)
port socketStatusReceiver : (Bool -> msg) -> Sub msg

-- Empfängt den Text-Inhalt einer importierten JSON-Datei
port configReceived : (String -> msg) -> Sub msg

-- Empfängt System-Nachrichten für das Log-Panel in der Sidebar
port logReceivedReceiver : (String -> msg) -> Sub msg