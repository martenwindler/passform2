module Types exposing (..)

import Dict exposing (Dict)
import Json.Decode as Decode
import Types.Domain exposing (..)


-- --- FLAGS ---

type alias Flags =
    { backendIP : String
    , savedConfig : Maybe String
    }


-- --- LAYOUT MODES ---

type LayoutMode
    = LandingMode
    | AppMode


-- --- MODEL ---

type alias Model =
    { activeLayout : LayoutMode 
    , mode : Mode
    , backendIP : String
    , connected : Bool
    , rosConnected : Bool
    , canConnected : Bool
    , rangerBattery : Maybe Float
    , agents : Dict (Int, Int) AgentModule
    -- --- NEU: Digital Twin & Logistics Data ---
    , bays : List Bay
    , inventory : List WorldItem
    , worldState : Dict String Decode.Value -- Rohdaten für den Planner
    -- -----------------------------------------
    , savedDefault : Dict (Int, Int) AgentModule
    , logs : List SystemLog
    , pathStart : Maybe GridCell
    , pathGoal : Maybe GridCell
    , currentPath : Maybe Path
    , hoveredCell : Maybe GridCell
    , editing : Bool
    , isDragging : Bool
    , is3D : Bool
    , loading : Bool
    , activeMenu : Maybe MenuType
    , sidebarOpen : Bool
    , activeSidebarTab : SidebarTab
    , gridWidth : Int
    , gridHeight : Int
    , waitingForNfc : Bool
    , nfcStatus : HardwareStatus
    , planningWeights : PlanningWeights
    , currentHz : Float
    , alert : Maybe String
    , connectedHardware : List HardwareDevice
    , lastWrittenId : Maybe String
    }


type MenuType
    = SelectionMenu GridCell
    | SettingsMenu GridCell AgentModule


-- --- MESSAGES (Dispatcher) ---

type Msg
    = PlanningMsg PlanningMsg
    | HardwareMsg HardwareMsg
    | AgentsMsg AgentsMsg
    | SystemMsg SystemMsg 
    | NoOp


-- --- DOMÄNEN-SPEZIFISCHE MESSAGES ---

type SystemMsg
    = NewProject                
    | OpenFileBrowser           
    | DragOver                  
    | FileDropped Decode.Value  
    | FileSelected Decode.Value 
    | ResetToLanding            
    -- NEU: Weltzustand für HTN Planner
    | HandleWorldState Decode.Value


type PlanningMsg
    = SetGridWidth String
    | SetGridHeight String
    | SetPathStart GridCell
    | SetPathGoal GridCell
    | SetWeight String String
    | SaveWeights
    | StartPlanning Bool
    | SetPathResult Path
    | PlanningResultRaw Decode.Value
    | ConfigReceived String


type HardwareMsg
    = SetConnected Bool
    | SetRosConnected Bool
    | HandleSystemLog (Result Decode.Error SystemLog)
    | HandleRfid (Result Decode.Error String)
    | HandleNfcStatus (Result Decode.Error String)
    -- --- NEU: Digital Twin Handlers ---
    | HandleInitialBays (Result Decode.Error (List Bay))
    | HandleBayUpdate (Result Decode.Error Bay)
    | HandleInventoryUpdate (Result Decode.Error (List WorldItem))
    | HandleSpecsUpdate Decode.Value
    | HandleRosInterfaces Decode.Value
    -- ----------------------------------
    | RequestNfcWrite String
    | SetMode String
    | ChangeHz Float
    | DismissAlert
    | HandleHardwareUpdate (Result Decode.Error (List HardwareDevice))
    | HandleRangerBattery Float


type AgentsMsg
    = ToggleMode
    | ToggleViewMode
    | ToggleSidebar
    | SwitchSidebarTab SidebarTab
    | CloseMenu
    | OpenMenu Int Int
    | SetCurrentAsDefault
    | LoadDefaultConfig
    | ClearGrid
    | ExportConfig
    | ImportConfigTrigger
    | StartAgent ModuleType GridCell
    | RemoveAgent GridCell
    | RotateAgent GridCell
    | UpdateAgent GridCell AgentModule
    | UpdateAgents Decode.Value
    | MoveAgent String GridCell
    | HandleGridClick GridCell
    | SetHoveredCell (Maybe GridCell)