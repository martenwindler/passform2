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

type alias AgentModule =
    { agent_id : Maybe String
    , module_type : String
    , position : GridCell
    , orientation : Int
    -- NEU: Markiert, ob es ein FTF oder ein statisches Modul ist
    , is_dynamic : Bool
    -- NEU: ID des aktuell transportierten Moduls (falls vorhanden)
    , payload : Maybe String
    }

type alias Path =
    { status : Int
    , cost : Float
    , path : List AgentModule
    }

-- --- MODEL ---

type alias Model =
    { mode : Mode
    , backendIP : String
    , connected : Bool
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
    | ExportConfig
    | ImportConfigTrigger
    | ConfigReceived String
    | StartAgent String GridCell
    | RemoveAgent GridCell
    | RotateAgent GridCell
    | MoveAgent { oldX : Int, oldY : Int, newX : Int, newY : Int }
    | UpdateAgents Decode.Value
    | SetConnected Bool
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