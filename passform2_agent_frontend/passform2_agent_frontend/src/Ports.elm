port module Ports exposing (..)

import Html exposing (Attribute) -- WICHTIG für onAgentMoved
import Html.Events exposing (on)
import Json.Decode as Decode
import Json.Encode as Encode
import Types exposing (..) -- Importiert GridCell, Msg, etc.


-- --- OUTGOING (Elm -> JS) ---
-- Diese Befehle senden Daten von Elm an JavaScript


port connectToBackend : String -> Cmd msg


port setMode : String -> Cmd msg


port triggerPlanning : Decode.Value -> Cmd msg


port saveToLocalStorage : String -> Cmd msg


port exportConfig : String -> Cmd msg


port importConfigTrigger : () -> Cmd msg


port writeNfcTrigger : String -> Cmd msg


-- NEU: Sendet die geänderten Planungs-Gewichte an das Backend
port savePlanningWeights : Decode.Value -> Cmd msg


{-| 
NEU: Ein generischer Port für Socket.IO Emissions.
Da Ports nur ein Argument erlauben, senden wir ein Tupel aus (EventName, JSON-Daten).
-}
port socketEmitPort : ( String, Encode.Value ) -> Cmd msg


{-| 
Hilfsfunktion, die in der Main.elm aufgerufen wird.
Macht den Aufruf sauberer: Ports.socketEmit "event_name" encodedData
-}
socketEmit : String -> Encode.Value -> Cmd msg
socketEmit eventName data =
    socketEmitPort ( eventName, data )


-- --- INCOMING (JS -> Elm) ---
-- Diese Subscriptions empfangen Daten von JavaScript in Elm


port socketStatusReceiver : (Bool -> msg) -> Sub msg


port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg


port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg


port systemLogReceiver : (Decode.Value -> msg) -> Sub msg


port rfidReceiver : (Decode.Value -> msg) -> Sub msg


port configReceived : (String -> msg) -> Sub msg


port nfcStatusReceiver : (Decode.Value -> msg) -> Sub msg

port hardwareUpdateReceiver : (Decode.Value -> msg) -> Sub msg

-- NEU: Event-Helper für die 3D-View
onAgentMoved : Attribute Msg
onAgentMoved =
    on "agent-moved" (Decode.map MoveAgent decodeAgentMove)

onCellClicked : Attribute Msg
onCellClicked =
    on "cell-clicked" (Decode.map HandleGridClick decodeCellClick)

-- Interne Decoder für die Events
decodeAgentMove : Decode.Decoder { oldX : Int, oldY : Int, newX : Int, newY : Int }
decodeAgentMove =
    Decode.at [ "detail" ] <|
        Decode.map4 (\ox oy nx ny -> { oldX = ox, oldY = oy, newX = nx, newY = ny })
            (Decode.field "oldX" Decode.int)
            (Decode.field "oldY" Decode.int)
            (Decode.field "newX" Decode.int)
            (Decode.field "newY" Decode.int)

decodeCellClick : Decode.Decoder GridCell
decodeCellClick =
    Decode.at ["detail"] 
        (Decode.map2 GridCell
            (Decode.field "x" Decode.int)
            (Decode.field "y" Decode.int)
        )