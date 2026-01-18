port module Ports exposing (..)

import Html exposing (Attribute)
import Html.Events exposing (on)
import Json.Decode as Decode
import Json.Encode as Encode
import Types exposing (..)
import Types.Domain exposing (..)


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

port socketStatusReceiver : (Bool -> msg) -> Sub msg

port rosStatusReceiver : (Bool -> msg) -> Sub msg

port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg

port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg

port systemLogReceiver : (Decode.Value -> msg) -> Sub msg

port rfidReceiver : (Decode.Value -> msg) -> Sub msg

port configReceived : (String -> msg) -> Sub msg

port nfcStatusReceiver : (Decode.Value -> msg) -> Sub msg

port hardwareUpdateReceiver : (Decode.Value -> msg) -> Sub msg


-- --- EVENT-HELPER FÜR DIE 3D-VIEW (KORRIGIERT) ---

{-| 
  Nimmt eine Funktion entgegen, die die Daten in eine Nachricht umwandelt.
  Dadurch wird der Helper flexibel für verschiedene Msg-Typen.
-}
onAgentMoved : ( { oldX : Int, oldY : Int, newX : Int, newY : Int } -> msg ) -> Attribute msg
onAgentMoved toMsg =
    on "agent-moved" (Decode.map toMsg decodeAgentMove)


onCellClicked : (GridCell -> msg) -> Attribute msg
onCellClicked toMsg =
    on "cell-clicked" (Decode.map toMsg decodeCellClick)


-- --- INTERNE DECODER (Bleiben gleich) ---

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