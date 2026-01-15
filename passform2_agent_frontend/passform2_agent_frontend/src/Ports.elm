port module Ports exposing (..)

import Json.Decode as Decode

-- --- OUTGOING (Elm -> JS) ---
-- Diese Namen müssen in JS mit .subscribe() aufgerufen werden
port connectToBackend : String -> Cmd msg
port setMode : String -> Cmd msg
port triggerPlanning : Decode.Value -> Cmd msg
port saveToLocalStorage : String -> Cmd msg
port exportConfig : String -> Cmd msg
port importConfigTrigger : () -> Cmd msg

-- --- INCOMING (JS -> Elm) ---
-- Diese Namen werden in JS mit .send() bedient
port socketStatusReceiver : (Bool -> msg) -> Sub msg
port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg
port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg
port systemLogReceiver : (Decode.Value -> msg) -> Sub msg
port rfidReceiver : (Decode.Value -> msg) -> Sub msg
port configReceived : (String -> msg) -> Sub msg