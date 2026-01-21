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
import Types.Domain exposing (..)

-- LOGIK-IMPORTE (Dispatcher-Zielmodule)
import Update.Planning as Planning
import Update.Hardware as Hardware
import Update.Agents as Agents
import Update.System as System

-- VIEW-IMPORTE (Das neue Layout-System)
import View.Organisms.Navbar as Navbar
import View.Organisms.Modal as Modal
import View.Organisms.Sidebar as Sidebar
import View.Layouts.LandingLayout as LandingLayout
import View.Layouts.LandingLayout as LandingLayout
import View.Organisms.Modal

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

init : Flags -> ( Model, Cmd Msg )
init flags =
    let
        initialAgents =
            Dict.empty -- Oder dein Standard-Gitter, falls vorhanden
    in
    ( { activeLayout = LandingMode -- HIER WAR DER FEHLER: Das Feld hat gefehlt
      , mode = Simulation
      , backendIP = flags.backendIP
      , connected = True
      , rosConnected = False
      , canConnected = False
      , rangerBattery = Nothing
      , agents = initialAgents
      , savedDefault = initialAgents
      , connectedHardware = []
      , logs = [ { message = "System bereit.", level = Success } ] 
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

        SystemMsg sMsg ->
            System.update sMsg model

        NoOp ->
            ( model, Cmd.none )

-- --- SUBSCRIPTIONS ---

subscriptions : Model -> Sub Msg
subscriptions model =
    Sub.batch
        [ Ports.socketStatusReceiver (SetConnected >> HardwareMsg)
        , Ports.rosStatusReceiver (SetRosConnected >> HardwareMsg)
        , Ports.activeAgentsReceiver (UpdateAgents >> AgentsMsg)
        , Ports.pathCompleteReceiver handlePathComplete
        , Ports.configReceived (ConfigReceived >> PlanningMsg)
        , Ports.systemLogReceiver (\val -> HardwareMsg (HandleSystemLog (Decode.decodeValue Decoders.decodeSystemLog val)))
        , Ports.rfidReceiver (\val -> HardwareMsg (HandleRfid (Decode.decodeValue Decoders.decodeRfid val)))
        , Ports.nfcStatusReceiver (\val -> HardwareMsg (HandleNfcStatus (Decode.decodeValue Decode.string val)))
        , Ports.hardwareUpdateReceiver (\val -> HardwareMsg (HandleHardwareUpdate (Decode.decodeValue Decoders.hardwareListDecoder val)))
        ]

{-| 
Hilfsfunktion, um das CNP-Ergebnis vom Port zu verarbeiten.
Nutzt den typsicheren Decoder und schickt das Ergebnis an SetPathResult.
-}
handlePathComplete : Decode.Value -> Msg
handlePathComplete rawValue =
    case Decode.decodeValue Decoders.decodePathResult rawValue of
        Ok validPath ->
            -- Wir nutzen die neue, typsichere Msg aus deiner Types.elm
            PlanningMsg (SetPathResult validPath)

        Err _ ->
            -- Im Fehlerfall machen wir nichts (oder man könnte einen Log-Eintrag schicken)
            NoOp


-- --- VIEW ---

view : Model -> Html Msg
view model =
    div [ class "app-shell" ]
        [ Navbar.view model
        , case model.activeLayout of
            LandingMode ->
                LandingLayout.view model

            AppMode ->
                div [ class "content-area" ] 
                    [ view3D model
                    , Sidebar.view model 
                    ]
        
        -- WICHTIG: Das Modal muss HIER stehen, damit es über der Landingpage 
        -- UND über dem Gitter angezeigt werden kann!
        , View.Organisms.Modal.view model 
        ]

-- --- HELPER VIEW (3D Scene Interop) ---

view3D : Model -> Html Msg
view3D model =
    let
        allowInteraction =
            case model.mode of
                Simulation ->
                    "true"

                Hardware ->
                    "false"
                    
        -- Wir geben dem Gitter explizit mit, in welchem Modus wir sind
        layoutModeStr =
            case model.activeLayout of
                LandingMode -> "landing"
                AppMode -> "app"
    in
    node "three-grid-scene"
        [ -- Basis-Konfiguration
          attribute "grid-width" (String.fromInt model.gridWidth)
        , attribute "grid-height" (String.fromInt model.gridHeight)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        
        -- Interaktions-Logik
        , attribute "allow-interaction" allowInteraction 
        , attribute "active-layout" layoutModeStr
        
        -- Daten-Objekte (als JSON-Strings für das Custom Element)
        , attribute "agents" (model.agents |> Decoders.encodeAgentMap |> Encode.encode 0)
        , attribute "path" (Decoders.encodePath model.currentPath |> Encode.encode 0)
        
        -- Positionen
        , attribute "start-pos" 
            (model.pathStart 
                |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) 
                |> Maybe.withDefault Encode.null 
                |> Encode.encode 0
            )
        , attribute "goal-pos" 
            (model.pathGoal 
                |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) 
                |> Maybe.withDefault Encode.null 
                |> Encode.encode 0
            )
            
        -- EVENT-LISTENER (Die Brücke zurück zu Elm)
        , Ports.onAgentMoved (MoveAgent >> AgentsMsg)
        , Ports.onCellClicked (HandleGridClick >> AgentsMsg) 
        ]
        []