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
    , agents : Dict (Int, Int, Int) AgentModule
    , bays : List Bay
    , inventory : List WorldItem
    , worldState : Dict String Decode.Value
    , savedDefault : Dict (Int, Int, Int) AgentModule
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
    | ToggleSidebar
    | SwitchSidebarTab SidebarTab
    | NoOp


-- --- DOMÄNEN-SPEZIFISCHE MESSAGES ---

type SystemMsg
    = NewProject                -- Startet mit leerem Gitter
    | SelectTemplate String     -- Lädt eine vordefinierte JSON
    | OpenFileBrowser           
    | DragOver                  
    | FileDropped Decode.Value  
    | FileSelected Decode.Value 
    | ResetToLanding            
    | HandleWorldState Decode.Value
    | EnterAppMode              -- Der explizite Wechsel ins Gitter
    | RotateCamera Float        -- NEU: Kamera-Rotation für Three.js


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
    | CloseMenu
    | OpenMenu Int Int
    | OpenSelectionMenu GridCell
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