module Types exposing (..)

import Dict exposing (Dict)
import Json.Decode as Decode
import Types.Domain exposing (..)


-- --- MODEL ---

type alias Model =
    { mode : Mode
    , backendIP : String
    , connected : Bool
    , rosConnected : Bool
    , canConnected : Bool
    , rangerBattery : Maybe Float
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
    | NoOp


-- --- DOMÄNEN-SPEZIFISCHE MESSAGES ---

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
    | MoveAgent { oldX : Int, oldY : Int, newX : Int, newY : Int }
    | HandleGridClick GridCell
    | SetHoveredCell (Maybe GridCell)