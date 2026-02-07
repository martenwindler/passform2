module Main exposing (main)

import Browser
import Decoders
import Dict exposing (Dict)
import Html exposing (..)
import Html.Attributes exposing (class, attribute) -- 'attribute' explizit importieren
import Html.Events exposing (on) -- 'on' für Custom Events importieren
import Json.Decode as Decode
import Json.Encode as Encode
import Ports
import Types exposing (..)
import Types.Domain exposing (..)
import Browser.Events

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
            Dict.empty
    in
    ( { activeLayout = LandingMode
      , mode = Simulation
      , backendIP = flags.backendIP
      , connected = True
      , rosConnected = False
      , canConnected = False
      , rangerBattery = Nothing
      , agents = initialAgents
      
      -- --- NEU: Diese 3 Felder fehlten ---
      , bays = []
      , inventory = []
      , worldState = Dict.empty
      -- -----------------------------------

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
      , isDragging = False
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
        ToggleSidebar ->
            ( { model | sidebarOpen = not model.sidebarOpen }, Cmd.none )

        SwitchSidebarTab tab ->
            ( { model | activeSidebarTab = tab, sidebarOpen = True }, Cmd.none )

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
        
        -- 1. Live-Updates für Agenten
        , Ports.activeAgentsReceiver (UpdateAgents >> AgentsMsg)
        
        -- 2. NEU: Digitaler Zwilling (Bays & Layout)
        , Ports.initialBaysReceiver (\val -> HardwareMsg (HandleInitialBays (Decode.decodeValue Decoders.decodeBayList val)))
        , Ports.bayUpdateReceiver (\val -> HardwareMsg (HandleBayUpdate (Decode.decodeValue Decoders.decodeBay val)))
        
        -- 3. NEU: Logistik & Weltzustand
        , Ports.inventoryReceiver (\val -> HardwareMsg (HandleInventoryUpdate (Decode.decodeValue Decoders.decodeInventory val)))
        , Ports.worldStateReceiver (HandleWorldState >> SystemMsg)
        , Ports.specsReceiver (HandleSpecsUpdate >> HardwareMsg)
        , Ports.rosInterfacesReceiver (HandleRosInterfaces >> HardwareMsg)

        -- 4. Datei-Inhalte (Konfigurations-Import)
        , Ports.configReceived (\jsonString -> 
            case Decode.decodeString Decode.value jsonString of
                Ok val -> AgentsMsg (UpdateAgents val)
                Err _ -> NoOp
          )
        , Ports.fileContentRead (\jsonString -> 
            case Decode.decodeString Decode.value jsonString of
                Ok val -> AgentsMsg (UpdateAgents val)
                Err _ -> NoOp
          )
        
        -- 5. System-Rückmeldungen & Hardware-Events
        , Ports.pathCompleteReceiver handlePathComplete
        , Ports.systemLogReceiver (\val -> HardwareMsg (HandleSystemLog (Decode.decodeValue Decoders.decodeSystemLog val)))
        , Ports.rfidReceiver (\val -> HardwareMsg (HandleRfid (Decode.decodeValue Decoders.decodeRfid val)))
        , Ports.nfcStatusReceiver (\val -> HardwareMsg (HandleNfcStatus (Decode.decodeValue Decode.string val)))
        , Ports.hardwareUpdateReceiver (\val -> HardwareMsg (HandleHardwareUpdate (Decode.decodeValue Decoders.hardwareListDecoder val)))
        
        -- 6. Interaktion aus der 3D-View
        , Ports.onAgentMoved (\d -> 
            AgentsMsg (MoveAgent d.agentId { x = d.newX, y = d.newY, z = 0, level = 0 })
          )

        , if model.activeLayout == AppMode then Browser.Events.onKeyDown keyDecoder else Sub.none

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
                Simulation -> "true"
                Hardware -> "false"

        layoutModeStr =
            case model.activeLayout of
                LandingMode -> "landing"
                AppMode -> "app"
    in
    node "three-grid-scene"
        [ -- 1. Einfache Flags bleiben Attribute (Strings sind hier ok)
          attribute "grid-width" (String.fromInt model.gridWidth)
        , attribute "grid-height" (String.fromInt model.gridHeight)
        , attribute "is-3d" (if model.is3D then "true" else "false")
        , attribute "allow-interaction" allowInteraction 
        , attribute "active-layout" layoutModeStr
        
        -- 2. KOMPLEXE DATEN als Property (Kein Encode.encode 0 nötig!)
        -- Das schickt das echte JS-Objekt direkt an die Three.js Klasse
        , Html.Attributes.property "agents" (Decoders.encodeAgentMap model.agents)
        , Html.Attributes.property "bays" (Decoders.encodeBayList model.bays)
        , Html.Attributes.property "currentPath" (model.currentPath |> Maybe.map Decoders.encodePath |> Maybe.withDefault Encode.null)
        
        -- 3. Positionen (Ebenfalls als Property für sauberes Mapping)
        , Html.Attributes.property "startPos" 
            (model.pathStart 
                |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) 
                |> Maybe.withDefault Encode.null 
            )
        , Html.Attributes.property "goalPos" 
            (model.pathGoal 
                |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) 
                |> Maybe.withDefault Encode.null 
            )

        -- --- EVENT LISTENER ---
        , Ports.onCellClicked (HandleGridClick >> AgentsMsg)
        , Html.Events.on "agent-moved" (Decode.at ["detail"] decodeMoveAgent)
        ]
        []
    
-- Hilfsfunktion für die Positionen, um die View sauber zu halten
encodePos : Maybe GridCell -> String
encodePos maybeCell =
    maybeCell 
        |> Maybe.map (\c -> Encode.object [ ( "x", Encode.int c.x ), ( "y", Encode.int c.y ) ]) 
        |> Maybe.withDefault Encode.null 
        |> Encode.encode 0

-- Der Decoder, der das JS-Event liest
decodeMoveAgent : Decode.Decoder Msg
decodeMoveAgent =
    Decode.map2 (\id cell -> AgentsMsg (MoveAgent id cell))
        (Decode.field "agentId" Decode.string)
        (Decode.map4 GridCell 
            (Decode.field "newX" Decode.int) 
            (Decode.field "newY" Decode.int)
            (Decode.succeed 0) -- z
            (Decode.field "level" Decode.int) 
        )

keyDecoder : Decode.Decoder Msg
keyDecoder =
    Decode.field "key" Decode.string
        |> Decode.map (\key ->
            -- Das loggt JEDE Taste, die du drückst
            let _ = Debug.log "Gedrückte Taste" key in
            case key of
                "ArrowLeft" -> SystemMsg (RotateCamera 0.1)
                "a"         -> SystemMsg (RotateCamera 0.1)
                "A"         -> SystemMsg (RotateCamera 0.1)
                "ArrowRight" -> SystemMsg (RotateCamera -0.1)
                "d"          -> SystemMsg (RotateCamera -0.1)
                "D"          -> SystemMsg (RotateCamera -0.1)
                _            -> NoOp
        )