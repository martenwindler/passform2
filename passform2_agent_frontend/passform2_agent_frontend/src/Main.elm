module Main exposing (main)

import Browser
import Decoders
import Dict exposing (Dict)
import Html exposing (..)
import Html.Attributes exposing (..)
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)
import Update 
import View.Navbar as Navbar
import View.Sidebar as Sidebar
import View.Modal as Modal 
import View.HardwareStatus as HardwareStatus 


-- PROGRAM


main : Program { backendIP : String, savedConfig : Maybe String } Model Msg
main =
    Browser.element
        { init = init
        , view = view
        , update = Update.update
        , subscriptions = subscriptions
        }


-- INIT


init : { backendIP : String, savedConfig : Maybe String } -> ( Model, Cmd Msg )
init flags =
    let
        initialAgents =
            case flags.savedConfig of
                Just json ->
                    case Decode.decodeString Decoders.agentMapDecoder json of
                        Ok agents -> agents
                        Err _ -> Dict.empty

                Nothing -> Dict.empty
    in
    ( { mode = Simulation
      , backendIP = flags.backendIP
      , connected = True
      , rosConnected = False
      , agents = initialAgents
      , savedDefault = initialAgents
      , connectedHardware = []
      , logs = [ { message = "System bereit. SSoT geladen.", level = "success" } ]
      , pathStart = Nothing
      , pathGoal = Nothing
      , currentPath = Nothing
      , hoveredCell = Nothing
      , sidebarOpen = False
      , gridWidth = 6
      , gridHeight = 4
      , editing = True
      , is3D = False
      , loading = False
      , activeMenu = Nothing
      , waitingForNfc = False
      , nfcStatus = "unknown"
      , currentHz = 1.0
      , alert = Nothing
      , planningWeights =
            { execution_time_default = 1.0
            , complex_module_time = 3.5
            , human_extra_weight = 1.0
            , proximity_penalty = 0.5
            , hardware_safety_factor = 1.2
            }
      }
    , Ports.connectToBackend flags.backendIP
    )


-- SUBSCRIPTIONS


subscriptions : Model -> Sub Msg
subscriptions model =
    Sub.batch
        [ Ports.socketStatusReceiver SetConnected
        , Ports.rosStatusReceiver SetRosConnected
        , Ports.activeAgentsReceiver UpdateAgents
        , Ports.pathCompleteReceiver PlanningResultRaw
        , Ports.configReceived ConfigReceived
        , Ports.systemLogReceiver (Decode.decodeValue Decoders.decodeSystemLog >> HandleSystemLog)
        , Ports.rfidReceiver (Decode.decodeValue Decoders.decodeRfid >> HandleRfid)
        , Ports.nfcStatusReceiver (Decode.decodeValue Decode.string >> HandleNfcStatus)
        , Ports.hardwareUpdateReceiver (Decode.decodeValue Decoders.hardwareListDecoder >> HandleHardwareUpdate)
        ]


-- VIEW


view : Model -> Html Msg
view model =
    div [ class "app-layout" ]
        [ Navbar.view model
        , div [ class "content-area" ]
            [ view3D model 
            , if model.sidebarOpen then Sidebar.view model else text ""
            , HardwareStatus.viewAlertOverlay model.alert 
            ]
        , Modal.viewActiveMenu model model.activeMenu 
        ]


-- HELPER VIEW (3D INTERFACE)


view3D : Model -> Html Msg
view3D model =
    node "three-grid-scene"
        [ attribute "agents" (model.agents |> Decoders.encodeAgentMap |> Encode.encode 0)
        , attribute "path" (Decoders.encodePath model.currentPath |> Encode.encode 0)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        , attribute "grid-width" (String.fromInt model.gridWidth)
        , attribute "grid-height" (String.fromInt model.gridHeight)
        , attribute "start-pos" (model.pathStart |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        , attribute "goal-pos" (model.pathGoal |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        , Ports.onAgentMoved 
        , Ports.onCellClicked 
        ]
        []