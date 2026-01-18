module Types exposing (..)

import Dict exposing (Dict)
import Http
import Json.Decode as Decode

-- --- ENTITIES ---

type alias GridCell = 
    { x : Int, y : Int }

type alias SystemLog =
    { message : String
    , level : String
    }

type alias HardwareDevice =
    { pi_id : String
    , rfid_status : String
    , pi_exists : Bool
    }

type alias AgentModule =
    { agent_id : Maybe String
    , module_type : String
    , position : GridCell
    , orientation : Int
    , is_dynamic : Bool
    , payload : Maybe String
    , signal_strength : Int -- 0-100% Signalqualität
    }

type alias Path =
    { status : Int
    , cost : Float
    , path : List AgentModule
    }

{-| 
Diese Gewichte definieren nach Emil Harlan die Kostenfunktion 
für das Contract-Net-Verfahren (CNP).
-}
type alias PlanningWeights =
    { execution_time_default : Float
    , complex_module_time : Float
    , human_extra_weight : Float
    , proximity_penalty : Float
    , hardware_safety_factor : Float
    }

-- --- MODEL ---

type alias Model =
    { mode : Mode
    , backendIP : String
    , connected : Bool             -- Status für Backend-REST (Port 8000)
    , rosConnected : Bool          -- Status für Backend-ROS (Port 5000)
    , canConnected : Bool          -- NEU: Status des CAN-Bus für den Ranger
    , rangerBattery : Maybe Float  -- NEU: Batteriespannung vom CAN-Bus
    , agents : Dict (Int, Int) AgentModule
    , savedDefault : Dict (Int, Int) AgentModule
    , logs : List SystemLog 
    , pathStart : Maybe GridCell
    , pathGoal : Maybe GridCell
    , currentPath : Maybe Path
    , hoveredCell : Maybe GridCell
    , editing : Bool
    , is3D : Bool
    , loading : Bool
    , activeMenu : Maybe MenuType
    , sidebarOpen : Bool
    , gridWidth : Int
    , gridHeight : Int
    , waitingForNfc : Bool
    , nfcStatus : String
    , planningWeights : PlanningWeights
    , currentHz : Float           -- Aktueller System-Takt (SSoT)
    , alert : Maybe String        -- ID des kritischen Agenten für Warn-Toast
    , connectedHardware : List HardwareDevice -- Externe RPis
    }

type Mode = Simulation | Hardware

type MenuType
    = SelectionMenu GridCell
    | SettingsMenu GridCell AgentModule

-- --- MESSAGES ---

type Msg
    = NoOp
    | ToggleMode
    | ToggleViewMode
    | ToggleSidebar
    | SetGridWidth String
    | SetGridHeight String
    | CloseMenu
    | SetCurrentAsDefault
    | LoadDefaultConfig
    | ClearGrid
    | SetMode String              -- FIX: Jetzt im Compiler bekannt (für Ranger_Manual etc.)
    | ExportConfig
    | ImportConfigTrigger
    | ConfigReceived String
    | StartAgent String GridCell
    | RemoveAgent GridCell
    | RotateAgent GridCell
    | MoveAgent { oldX : Int, oldY : Int, newX : Int, newY : Int }
    | UpdateAgents Decode.Value
    | SetConnected Bool           -- Setzt REST-Status
    | SetRosConnected Bool        -- Setzt ROS-Status
    | LogReceived String
    | HandleGridClick GridCell
    | SetPathStart GridCell
    | SetPathGoal GridCell
    | StartPlanning Bool
    | PlanningResult (Result Http.Error Path)
    | PlanningResultRaw Decode.Value
    | ModeChanged (Result Http.Error ())
    | HandleSystemLog (Result Decode.Error SystemLog)
    | HandleRfid (Result Decode.Error String)
    | HandleNfcStatus (Result Decode.Error String)
    | RequestNfcWrite String
    | SetWeight String String 
    | SaveWeights
    | ChangeHz Float                 
    | DismissAlert
    | HandleHardwareUpdate (Result Decode.Error (List HardwareDevice))