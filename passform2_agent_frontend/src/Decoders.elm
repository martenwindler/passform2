module Decoders exposing (..)

import Types exposing (..)
import Types.Domain exposing (..)
import Dict exposing (Dict)
import Json.Decode as Decode exposing (Decoder, bool, field, float, int, list, maybe, string)
import Json.Encode as Encode

-- --- TYPE-SAFE MAPPERS (Internal) ---


moduleTypeDecoder : Decoder ModuleType
moduleTypeDecoder =
    string |> Decode.map (\s ->
        case String.toLower s of
            "ftf" -> FTF
            "ranger" -> FTF
            "conveyeur" -> Conveyeur
            "conveyor" -> Conveyeur
            "rollen_ns" -> RollenModul
            "rollenmodul" -> RollenModul
            "mensch" -> Mensch
            "human_operator" -> Mensch
            "greifer" -> Greifer
            "ur5__gripper" -> Greifer
            "tisch" -> Station
            "station" -> Station
            "y-module" -> Station
            "ur5" -> Station
            _ -> UnknownModule s
    )


logLevelDecoder : Decoder LogLevel
logLevelDecoder =
    string
        |> Decode.map
            (\s ->
                case s of
                    "success" ->
                        Success

                    "warning" ->
                        Warning

                    "error" ->
                        Danger

                    _ ->
                        Info
            )


hardwareStatusDecoder : Decoder HardwareStatus
hardwareStatusDecoder =
    string
        |> Decode.map
            (\s ->
                case String.toLower s of
                    "online" -> Online
                    "ok" -> Online     -- Rust sendet oft "Ok"
                    "missing" -> Missing
                    "error" -> Error
                    _ -> UnknownStatus
            )

moduleTypeToString : ModuleType -> String
moduleTypeToString mt =
    case mt of
        FTF ->
            "ftf"

        Conveyeur ->
            "conveyeur"

        RollenModul ->
            "rollen_ns"

        Mensch ->
            "mensch"

        Greifer ->
            "greifer"

        Station ->
            "tisch"

        UnknownModule s ->
            s

-- --- SYSTEM DECODERS ---


decodeSystemLog : Decoder SystemLog
decodeSystemLog =
    Decode.map2 SystemLog
        (field "message" string)
        (field "level" logLevelDecoder)


decodeRfid : Decoder String
decodeRfid =
    Decode.oneOf
        [ field "rfid_id" string
        , string
        ]


-- --- AGENT DECODERS ---


-- Ersetze alle Varianten von gridCellDecoder / decodeCell durch diese Logik:
gridCellDecoder : Decoder GridCell
gridCellDecoder =
    Decode.map4 GridCell
        (Decode.oneOf [ Decode.field "x" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "y" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "z" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "level" Decode.int, Decode.field "lvl" Decode.int, Decode.succeed 0 ])


andMap : Decoder a -> Decoder (a -> b) -> Decoder b
andMap decoderA decoderFunction =
    Decode.map2 (\value function -> function value) decoderA decoderFunction


-- AgentModuleDecoder anpassen (da jetzt 'status' im Typ steht!)
agentModuleDecoder : Decoder AgentModule
agentModuleDecoder =
    Decode.succeed AgentModule
        |> andMap (field "agent_id" (maybe string))
        |> andMap (field "module_type" moduleTypeDecoder)
        |> andMap (field "position" gridCellDecoder)
        |> andMap (Decode.oneOf [ field "orientation" int, Decode.succeed 0 ])
        |> andMap (Decode.oneOf [ field "is_dynamic" bool, Decode.succeed False ])
        |> andMap (Decode.oneOf [ field "payload" (maybe string), Decode.succeed Nothing ])
        |> andMap (Decode.oneOf [ field "signal_strength" int, Decode.succeed 100 ])
        |> andMap (field "status" hardwareStatusDecoder) -- NEU hinzugefügt!

-- Ersetze deinen agentMapDecoder in Decoders.elm durch diesen:

agentMapDecoder : Decoder (Dict (Int, Int, Int) AgentModule)
agentMapDecoder =
    Decode.oneOf
        [ -- Fall 1: Backend schickt { "agents": [...] }
          Decode.field "agents" decodeAgentListAsDict
        , -- Fall 2: Backend schickt direkt eine Liste [...]
          decodeAgentListAsDict
        , -- Fall 3: Backend schickt eine Map { "1,1,0": {...} }
          decodeAgentDictDirectly
        ]


-- HILFSFUNKTIONEN FÜR DEN DECODER --

decodeAgentListAsDict : Decoder (Dict (Int, Int, Int) AgentModule)
decodeAgentListAsDict =
    Decode.list agentDecoder
        |> Decode.map (\list -> 
            list 
                |> List.map (\a -> ( (a.position.x, a.position.y, a.position.level), a ))
                |> Dict.fromList
        )
        
decodeAgentDictDirectly : Decoder (Dict (Int, Int, Int) AgentModule)
decodeAgentDictDirectly =
    Decode.dict agentDecoder
        |> Decode.map (\dict ->
            dict
                |> Dict.values
                |> List.map (\a -> ( (a.position.x, a.position.y, a.position.level), a ))
                |> Dict.fromList
        )

{-| Der zentrale Decoder für ein Agenten-Modul (verarbeitet das flache Rust-Format) -}
agentDecoder : Decoder AgentModule
agentDecoder =
    Decode.succeed AgentModule
        |> andMap (Decode.maybe (Decode.field "agent_id" Decode.string))
        |> andMap (Decode.field "module_type" moduleTypeDecoder)
        |> andMap gridCellDecoder 
        |> andMap (Decode.oneOf [ Decode.field "orientation" Decode.int, Decode.succeed 0 ])
        |> andMap (Decode.oneOf [ Decode.field "is_dynamic" Decode.bool, Decode.succeed False ])
        |> andMap (Decode.oneOf [ Decode.field "payload" (Decode.maybe Decode.string), Decode.succeed Nothing ])
        |> andMap (Decode.oneOf [ Decode.field "signal_strength" Decode.int, Decode.succeed 100 ])
        |> andMap (Decode.oneOf [ Decode.field "status" hardwareStatusDecoder, Decode.succeed Online ])

-- Hilfsfunktion: Versucht x/y flach ODER in einem position-Objekt zu finden
decodeFlexiblePosition : Decoder GridCell
decodeFlexiblePosition =
    Decode.oneOf
        [ Decode.field "position" decodeGridCell
        , decodeGridCell
        ]

-- Neue Hilfsfunktion, um x/y flach in ein GridCell zu verwandeln
decodePositionFromFlat : Decoder GridCell
decodePositionFromFlat = decodeGridCell

decodeGridCell : Decoder GridCell
decodeGridCell =
    Decode.map4 GridCell
        (Decode.oneOf [ Decode.field "x" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "y" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "z" Decode.int, Decode.succeed 0 ])
        (Decode.oneOf [ Decode.field "level" Decode.int, Decode.field "lvl" Decode.int, Decode.succeed 0 ])

-- Hilfsfunktion um "0,0" zu (0,0) zu machen
parseKey : String -> (Int, Int)
parseKey str =
    case String.split "," str of
        [ xStr, yStr ] ->
            ( String.toInt xStr |> Maybe.withDefault 0
            , String.toInt yStr |> Maybe.withDefault 0
            )
        _ ->
            ( 0, 0 )

decodeCell : Decoder GridCell
decodeCell = decodeGridCell

hardwareDeviceDecoder : Decode.Decoder HardwareDevice
hardwareDeviceDecoder =
    Decode.map3 HardwareDevice
        (Decode.field "pi_id" Decode.string)
        (Decode.field "rfid_status" hardwareStatusDecoder)
        (Decode.field "pi_exists" Decode.bool)


hardwareListDecoder : Decode.Decoder (List HardwareDevice)
hardwareListDecoder =
    Decode.list hardwareDeviceDecoder


decodeModuleType =
    Decode.string
        |> Decode.andThen
            (\str ->
                let
                    s = String.toLower str
                in
                if String.contains "tisch" s then
                    Decode.succeed Station
                else
                    case s of
                        "ftf" -> Decode.succeed FTF
                        "ranger" -> Decode.succeed FTF
                        "conveyeur" -> Decode.succeed Conveyeur
                        "rollenmodul" -> Decode.succeed RollenModul
                        "mensch" -> Decode.succeed Mensch
                        "greifer" -> Decode.succeed Greifer
                        "station" -> Decode.succeed Station
                        _ -> Decode.succeed (UnknownModule str)
            )

-- --- PLANNING & PATH DECODERS ---

decodePathResult : Decoder Path
decodePathResult =
    Decode.map3 Path
        (Decode.oneOf [ field "status" int, Decode.succeed 200 ])
        (field "cost" float)
        -- KORREKTUR: Hier stand agentModuleDecoder, es muss gridCellDecoder sein!
        (field "path" (list gridCellDecoder))


planningWeightsDecoder : Decode.Decoder PlanningWeights
planningWeightsDecoder =
    Decode.map5 PlanningWeights
        (Decode.field "execution_time_default" Decode.float)
        (Decode.field "complex_module_time" Decode.float)
        (Decode.field "human_extra_weight" Decode.float)
        (Decode.field "proximity_penalty" Decode.float)
        (Decode.field "hardware_safety_factor" Decode.float)


pathDecoder : Decode.Decoder Path
pathDecoder =
    decodePathResult

-- --- ENCODERS ---

{-| NEU: Erzeugt das JSON für eine CNP-Aufgabenankündigung.
    Inklusive Agents-Liste, damit das Backend die Weltkarte kennt.
-}
encodeCnpAnnouncement : Model -> Encode.Value
encodeCnpAnnouncement model =
    Encode.object
        [ ( "type", Encode.string "CNP_TASK_ANNOUNCEMENT" )
        , ( "payload"
          , Encode.object
                [ ( "start", model.pathStart |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "goal", model.pathGoal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
                , ( "weights", encodeWeights model.planningWeights )
                , ( "agents", encodeAgentMap model.agents )
                ]
          )
        ]

encodeAgentMap : Dict (Int, Int, Int) AgentModule -> Encode.Value
encodeAgentMap agents =
    Encode.list encodeAgent (Dict.values agents)

encodeAgentModule : AgentModule -> Encode.Value
encodeAgentModule agent =
    Encode.object
        [ ( "agent_id", agent.agent_id |> Maybe.map Encode.string |> Maybe.withDefault Encode.null )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) )
        , ( "x", Encode.int agent.position.x )
        , ( "y", Encode.int agent.position.y )
        , ( "z", Encode.int agent.position.z )      -- NEU
        , ( "level", Encode.int agent.position.level ) -- NEU
        , ( "position", encodeGridCell agent.position )
        , ( "orientation", Encode.int agent.orientation )
        , ( "is_dynamic", Encode.bool agent.is_dynamic )
        , ( "payload", agent.payload |> Maybe.map Encode.string |> Maybe.withDefault Encode.null )
        , ( "signal_strength", Encode.int agent.signal_strength )
        ]

encodeGridCell : GridCell -> Encode.Value
encodeGridCell cell =
    Encode.object
        [ ( "x", Encode.int cell.x )
        , ( "y", Encode.int cell.y )
        , ( "z", Encode.int cell.z )
        , ( "level", Encode.int cell.level )
        ]

encodePath : Path -> Encode.Value
encodePath p =
    Encode.object
        [ ( "status", Encode.int p.status )
        , ( "cost", Encode.float p.cost )
        , ( "path", Encode.list encodeGridCell p.path ) 
        ]

encodeWeights : PlanningWeights -> Encode.Value
encodeWeights w =
    Encode.object
        [ ( "execution_time_default", Encode.float w.execution_time_default )
        , ( "complex_module_time", Encode.float w.complex_module_time )
        , ( "human_extra_weight", Encode.float w.human_extra_weight )
        , ( "proximity_penalty", Encode.float w.proximity_penalty )
        , ( "hardware_safety_factor", Encode.float w.hardware_safety_factor )
        ]


encodePlanningData : { start : Maybe GridCell, goal : Maybe GridCell, weights : PlanningWeights, isRanger : Bool } -> Encode.Value
encodePlanningData data =
    Encode.object
        [ ( "start", data.start |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "goal", data.goal |> Maybe.map encodeGridCell |> Maybe.withDefault Encode.null )
        , ( "weights", encodeWeights data.weights )
        , ( "isRanger", Encode.bool data.isRanger )
        ]

-- Encoder für die gesamte Konfiguration
encodeFullConfig : Model -> Encode.Value
encodeFullConfig model =
    Encode.object
        [ ( "config"
          , Encode.object 
                [ ( "grid"
                  , Encode.object 
                        [ ( "width", Encode.int model.gridWidth )
                        , ( "height", Encode.int model.gridHeight )
                        ]
                  )
                ]
          )
        , ( "agents", encodeAgentMap model.agents )
        , ( "bays", Encode.list encodeBay model.bays )
        -- inventory und worldState können wir mitschicken, Rust ignoriert sie jetzt
        , ( "inventory", encodeInventory model.inventory )
        , ( "worldState", encodeInventory model.inventory )
        ]
        
-- Hilfs-Encoder für das Inventar
encodeInventory : List WorldItem -> Encode.Value
encodeInventory items =
    Encode.list encodeWorldItem items

encodeWorldItem : WorldItem -> Encode.Value
encodeWorldItem item =
    Encode.object
        [ ( "name", Encode.string item.name )
        , ( "uid", Encode.string item.uid )
        , ( "quantity", Encode.int item.quantity )
        , ( "location", encodeWorldLocation item.location )
        ]

encodeWorldLocation : WorldLocation -> Encode.Value
encodeWorldLocation loc =
    Encode.object
        [ ( "frame_id", Encode.string loc.frame_id )
        , ( "pose", encodePose loc.pose )
        ]

encodePose : Pose -> Encode.Value
encodePose pose =
    Encode.object
        [ ( "position", encodePoint pose.position )
        , ( "orientation", encodeQuaternion pose.orientation )
        ]

encodePoint : Point -> Encode.Value
encodePoint p =
    Encode.object
        [ ( "x", Encode.float p.x )
        , ( "y", Encode.float p.y )
        , ( "z", Encode.float p.z )
        ]

encodeQuaternion : Quaternion -> Encode.Value
encodeQuaternion q =
    Encode.object
        [ ( "x", Encode.float q.x )
        , ( "y", Encode.float q.y )
        , ( "z", Encode.float q.z )
        , ( "w", Encode.float q.w )
        ]

encodeAgent : AgentModule -> Encode.Value
encodeAgent agent =
    Encode.object
        [ ( "agent_id", Encode.string (Maybe.withDefault "Unknown-ID" agent.agent_id) )
        , ( "module_type", Encode.string (moduleTypeToString agent.module_type) )
        , ( "x", Encode.int agent.position.x )
        , ( "y", Encode.int agent.position.y )
        , ( "z", Encode.int agent.position.z )
        , ( "level", Encode.int agent.position.level )
        , ( "position", encodeGridCell agent.position )
        , ( "orientation", Encode.int agent.orientation )
        , ( "is_dynamic", Encode.bool agent.is_dynamic )
        , ( "signal_strength", Encode.int agent.signal_strength )
        ]


{-| Hilfsdecoder: Verwandelt [3.0, 1.0, 0.0] in ein Point-Record -}
decodeOriginAsPoint : Decoder Point
decodeOriginAsPoint =
    Decode.list float
        |> Decode.andThen
            (\coords ->
                case coords of
                    [ x, y, z ] ->
                        Decode.succeed { x = x, y = y, z = z }

                    _ ->
                        Decode.fail "Origin muss eine Liste mit genau 3 Floats sein."
            )

{-| Der Haupt-Decoder für die Bays -}
-- Bay Decoder anpassen
decodeBay : Decoder Bay
decodeBay =
    Decode.succeed Bay
        |> andMap (field "unique_id" string)
        |> andMap (field "name" string)
        |> andMap (field "origin" decodeOriginAsPoint)
        |> andMap (field "is_virtual" bool)
        |> andMap (field "status" hardwareStatusDecoder)
        |> andMap (field "occupation" bool)
        |> andMap (field "module_uuid" string)
        |> andMap (Decode.oneOf [ field "level" int, Decode.succeed 0 ])

{-| Hilfsdecoder für das [x, y, z] Array aus Rust -}
decodeOrigin : Decoder { x : Float, y : Float, z : Float }
decodeOrigin =
    Decode.list float
        |> Decode.andThen
            (\l ->
                case l of
                    [ x, y, z ] -> Decode.succeed { x = x, y = y, z = z }
                    _ -> Decode.fail "Origin muss eine Liste mit genau 3 Floats sein [x, y, z]"
            )

decodeBayList : Decoder (List Bay)
decodeBayList =
    Decode.list decodeBay

{-| Hilfsfunktion für die Liste der Inventar-Items -}
decodeInventory : Decoder (List WorldItem)
decodeInventory =
    Decode.list decodeWorldItem

{-| Decoder für ein einzelnes Welt-Item (Inventory) -}
decodeWorldItem : Decoder WorldItem
decodeWorldItem =
    Decode.succeed WorldItem
        |> andMap (field "name" string)
        |> andMap (field "uid" string)
        |> andMap (field "quantity" int)
        |> andMap (field "location" decodeWorldLocation)

decodeWorldLocation : Decoder WorldLocation
decodeWorldLocation =
    Decode.succeed WorldLocation
        |> andMap (field "frame_id" string)
        |> andMap (field "pose" decodePose)

decodePose : Decoder Pose
decodePose =
    Decode.map2 Pose
        (field "position" decodePoint)
        (field "orientation" decodeQuaternion)

decodePoint : Decoder Point
decodePoint =
    Decode.map3 Point
        (field "x" float)
        (field "y" float)
        (field "z" float)

decodeQuaternion : Decoder Quaternion
decodeQuaternion =
    Decode.map4 Quaternion
        (field "x" float)
        (field "y" float)
        (field "z" float)
        (field "w" float)

encodeBayList : List Bay -> Encode.Value
encodeBayList bays =
    Encode.list encodeBay bays

encodeBay : Bay -> Encode.Value
encodeBay bay =
    Encode.object
        [ ( "unique_id", Encode.string bay.unique_id )
        , ( "name", Encode.string bay.name )
        , ( "x", Encode.float bay.origin.x )
        , ( "y", Encode.float bay.origin.y )
        , ( "occupation", Encode.bool bay.occupation )
        , ( "status", Encode.string "Ok" ) -- Oder dein Status-Mapper
        ]