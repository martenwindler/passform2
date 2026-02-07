port module Ports exposing (..)

import Html exposing (Attribute)
import Html.Events exposing (on)
import Json.Decode as Decode
import Json.Encode as Encode
import Types exposing (..)
import Types.Domain exposing (..)


-- --- OUTGOING (Elm -> JS: Befehle an das Backend/System) ---

port connectToBackend : String -> Cmd msg

port setMode : String -> Cmd msg

port triggerPlanning : Decode.Value -> Cmd msg

{-| Sendet die CNP-Ausschreibung (Task Announcement) an das Backend/ROS -}
port sendCnpRequest : Encode.Value -> Cmd msg

port saveToLocalStorage : String -> Cmd msg

port exportConfig : String -> Cmd msg

port importConfigTrigger : () -> Cmd msg

-- Schickt das File-Objekt (JSON) an JS zum Auslesen
port requestFileRead : Decode.Value -> Cmd msg

port writeNfcTrigger : String -> Cmd msg

port savePlanningWeights : Decode.Value -> Cmd msg

port pushConfig : Encode.Value -> Cmd msg

-- NEU: Schreibt die aktuellen Session-Daten permanent in den Golden Master (initial_00.json)
port persistToMaster : Encode.Value -> Cmd msg

-- Universeller Port für Socket.io Emits
port socketEmitPort : ( String, Encode.Value ) -> Cmd msg

socketEmit : String -> Encode.Value -> Cmd msg
socketEmit eventName data =
    socketEmitPort ( eventName, data )


-- --- INCOMING (JS -> Elm: Daten vom Backend/Hardware) ---

-- NEU: Digitaler Zwilling & Layout (Spatial Mapping)
port initialBaysReceiver : (Decode.Value -> msg) -> Sub msg

port bayUpdateReceiver : (Decode.Value -> msg) -> Sub msg

-- NEU: Logistik & Inventar (BaSyx/InventoryManager)
port inventoryReceiver : (Decode.Value -> msg) -> Sub msg

-- NEU: HTN Planner & Weltzustand
port worldStateReceiver : (Decode.Value -> msg) -> Sub msg

-- NEU: Hardware Spezifikationen & ROS-Interfaces
port specsReceiver : (Decode.Value -> msg) -> Sub msg

port rosInterfacesReceiver : (Decode.Value -> msg) -> Sub msg

-- BESTAND:
port skillsReceiver : (Decode.Value -> msg) -> Sub msg

port socketStatusReceiver : (Bool -> msg) -> Sub msg

port rosStatusReceiver : (Bool -> msg) -> Sub msg

port activeAgentsReceiver : (Decode.Value -> msg) -> Sub msg

{-| Empfängt das Ergebnis der Verhandlung (den Pfad des Gewinner-Agenten) -}
port pathCompleteReceiver : (Decode.Value -> msg) -> Sub msg

port systemLogReceiver : (Decode.Value -> msg) -> Sub msg

port rfidReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt den String-Inhalt der eingelesenen Datei
port fileContentRead : (String -> msg) -> Sub msg

port configReceived : (String -> msg) -> Sub msg

port nfcStatusReceiver : (Decode.Value -> msg) -> Sub msg

port hardwareUpdateReceiver : (Decode.Value -> msg) -> Sub msg

-- Empfängt Agenten-Bewegungen aus der 3D-View
port onAgentMoved : ({ agentId : String, oldX : Int, oldY : Int, newX : Int, newY : Int, level : Int } -> msg) -> Sub msg

{-| Schickt einen Rotationswert (z.B. 0.1 oder -0.1) an Three.js -}
port rotateCamera : Float -> Cmd msg

-- --- EVENT-HELPER FÜR DIE 3D-VIEW ---

onCellClicked : (GridCell -> msg) -> Attribute msg
onCellClicked toMsg =
    -- Elm hört auf das JS-Custom-Event "cell-clicked"
    on "cell-clicked" (Decode.map toMsg decodeCellClick)


-- --- INTERNE DECODER ---

{-| Zerlegt das CustomEvent "agent-moved" - jetzt inklusive Level -}
decodeAgentMove : Decode.Decoder { agentId : String, oldX : Int, oldY : Int, newX : Int, newY : Int, level : Int }
decodeAgentMove =
    Decode.at [ "detail" ] <|
        Decode.map6 (\id ox oy nx ny lvl -> { agentId = id, oldX = ox, oldY = oy, newX = nx, newY = ny, level = lvl })
            (Decode.field "agentId" Decode.string)
            (Decode.field "oldX" Decode.int)
            (Decode.field "oldY" Decode.int)
            (Decode.field "newX" Decode.int)
            (Decode.field "newY" Decode.int)
            (Decode.field "level" Decode.int)

{-| Zerlegt das CustomEvent "cell-clicked" aus der ThreeGridScene -}
decodeCellClick : Decode.Decoder GridCell
decodeCellClick =
    Decode.at [ "detail" ]
        (Decode.map4 GridCell
            (Decode.field "x" Decode.int)
            (Decode.field "y" Decode.int)
            (Decode.succeed 0) -- Klick kommt vom Boden -> z = 0
            (Decode.succeed 0) -- Klick kommt vom Boden -> level = 0
        )