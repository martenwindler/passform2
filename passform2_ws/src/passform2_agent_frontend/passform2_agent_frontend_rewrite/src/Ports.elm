port module Ports exposing (..)

import Json.Decode as Decode
import Json.Encode as Encode

-- OUTGOING (Elm -> JavaScript)
-- Sagt JS, dass es sich mit einer neuen IP verbinden soll (reconnect)
port connectToBackend : String -> Cmd msg

-- Sagt der 3D-Szene, ob sie 2D oder 3D anzeigen soll
port setViewMode : Bool -> Cmd msg


-- INCOMING (JavaScript -> Elm)
-- Empfängt die Liste der Agenten vom Socket.IO "active_agents" Event
port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt den fertigen Pfad vom Socket.IO "path_complete" Event
port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt den Verbindungsstatus (True = verbunden)
port socketStatusReceiver : (Bool -> msg) -> Sub msg