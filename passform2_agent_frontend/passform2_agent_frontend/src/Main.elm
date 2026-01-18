module Main exposing (main)

import Browser
import Decoders
import Dict exposing (Dict)
-- ODER (entsprechend deiner Pfadangabe):
import Html exposing (..)
import Html.Attributes exposing (..)
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)
import Types.Domain exposing (..)

-- LOGIK-IMPORTE
import Update.Planning as Planning
import Update.Hardware as Hardware
import Update.Agents as Agents

-- VIEW-IMPORTE
import View.Layouts.MainLayout as MainLayout


-- --- PROGRAM ---

main : Program { backendIP : String, savedConfig : Maybe String } Model Msg
main =
    Browser.element
        { init = init
        , view = view
        , update = update
        , subscriptions = subscriptions
        }


-- --- INIT ---

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
      , canConnected = False
      , rangerBattery = Nothing
      , agents = initialAgents
      , savedDefault = initialAgents
      , connectedHardware = []
      , logs = [ { message = "System bereit. ", level = Success } ] 
      , pathStart = Nothing
      , pathGoal = Nothing
      , currentPath = Nothing
      , hoveredCell = Nothing
      , sidebarOpen = False
      , activeSidebarTab = TabAgents 
      , gridWidth = 6
      , gridHeight = 4
      , editing = True
      , is3D = False
      , loading = False
      , activeMenu = Nothing
      , waitingForNfc = False
      , nfcStatus = UnknownStatus 
      , lastWrittenId = Nothing 
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


-- --- UPDATE (Dispatcher) ---

update : Msg -> Model -> ( Model, Cmd Msg )
update msg model =
    case msg of
        PlanningMsg pMsg ->
            Planning.update pMsg model

        HardwareMsg hMsg ->
            Hardware.update hMsg model

        AgentsMsg aMsg ->
            Agents.update aMsg model

        NoOp ->
            ( model, Cmd.none )


-- --- SUBSCRIPTIONS (Gemappt auf Domänen) ---

subscriptions : Model -> Sub Msg
subscriptions model =
    Sub.batch
        [ Ports.socketStatusReceiver (SetConnected >> HardwareMsg)
        , Ports.rosStatusReceiver (SetRosConnected >> HardwareMsg)
        , Ports.activeAgentsReceiver (UpdateAgents >> AgentsMsg)
        , Ports.pathCompleteReceiver (PlanningResultRaw >> PlanningMsg)
        , Ports.configReceived (ConfigReceived >> PlanningMsg)
        -- Decodierung erfolgt sicher in der Subscription-Schleife
        , Ports.systemLogReceiver (\val -> HardwareMsg (HandleSystemLog (Decode.decodeValue Decoders.decodeSystemLog val)))
        , Ports.rfidReceiver (\val -> HardwareMsg (HandleRfid (Decode.decodeValue Decoders.decodeRfid val)))
        , Ports.nfcStatusReceiver (\val -> HardwareMsg (HandleNfcStatus (Decode.decodeValue Decode.string val)))
        , Ports.hardwareUpdateReceiver (\val -> HardwareMsg (HandleHardwareUpdate (Decode.decodeValue Decoders.hardwareListDecoder val)))
        ]


-- --- VIEW ---

view : Model -> Html Msg
view model =
    MainLayout.view model (view3D model)


-- --- HELPER VIEW (3D INTERFACE) ---

view3D : Model -> Html Msg
view3D model =
    let
        allowInteraction =
            case model.mode of
                Simulation -> "true"
                Hardware -> "false"
    in
    node "three-grid-scene"
        [ attribute "agents" (model.agents |> Decoders.encodeAgentMap |> Encode.encode 0)
        , attribute "path" (Decoders.encodePath model.currentPath |> Encode.encode 0)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        , attribute "grid-width" (String.fromInt model.gridWidth)
        , attribute "grid-height" (String.fromInt model.gridHeight)
        , attribute "allow-interaction" allowInteraction 
        , attribute "start-pos" (model.pathStart |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        , attribute "goal-pos" (model.pathGoal |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) |> Maybe.withDefault Encode.null |> Encode.encode 0)
        -- Ports rufen nun Funktionen auf, die Nachrichten generieren
        , Ports.onAgentMoved (MoveAgent >> AgentsMsg)
        , Ports.onCellClicked (HandleGridClick >> AgentsMsg) 
        ]
        []