port module Ports exposing (..)

import Html exposing (Attribute)
import Html.Events exposing (on)
import Json.Decode as Decode
import Json.Encode as Encode
import Types exposing (..)


-- --- OUTGOING (Elm -> JS) ---

port connectToBackend : String -> Cmd msg

port setMode : String -> Cmd msg

port triggerPlanning : Decode.Value -> Cmd msg

port saveToLocalStorage : String -> Cmd msg

port exportConfig : String -> Cmd msg

port importConfigTrigger : () -> Cmd msg

port writeNfcTrigger : String -> Cmd msg

port savePlanningWeights : Decode.Value -> Cmd msg

port socketEmitPort : ( String, Encode.Value ) -> Cmd msg

socketEmit : String -> Encode.Value -> Cmd msg
socketEmit eventName data =
    socketEmitPort ( eventName, data )


-- --- INCOMING (JS -> Elm) ---

-- Status für das Haupt-Backend (Port 8000)
port socketStatusReceiver : (Bool -> msg) -> Sub msg

-- NEU: Status für die ROS-Bridge (Port 5000)
port rosStatusReceiver : (Bool -> msg) -> Sub msg

port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg

port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg

port systemLogReceiver : (Decode.Value -> msg) -> Sub msg

port rfidReceiver : (Decode.Value -> msg) -> Sub msg

port configReceived : (String -> msg) -> Sub msg

port nfcStatusReceiver : (Decode.Value -> msg) -> Sub msg

port hardwareUpdateReceiver : (Decode.Value -> msg) -> Sub msg


-- --- EVENT-HELPER FÜR DIE 3D-VIEW ---

onAgentMoved : Attribute Msg
onAgentMoved =
    on "agent-moved" (Decode.map MoveAgent decodeAgentMove)

onCellClicked : Attribute Msg
onCellClicked =
    on "cell-clicked" (Decode.map HandleGridClick decodeCellClick)


-- --- INTERNE DECODER ---

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