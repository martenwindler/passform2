module Types exposing (..)

import Dict exposing (Dict)
import Http
import Json.Decode as Decode

type alias Model =
    { mode : Mode
    , backendIP : String
    , connected : Bool
    , agents : Dict (Int, Int) AgentModule
    , pathStart : Maybe GridCell
    , pathGoal : Maybe GridCell
    , currentPath : Maybe Path
    , hoveredCell : Maybe GridCell
    , editing : Bool
    , is3D : Bool
    , loading : Bool
    , activeMenu : Maybe MenuType
    }

type Mode = Simulation | Hardware

type MenuType
    = SelectionMenu GridCell
    | SettingsMenu GridCell AgentModule

type alias GridCell = { x : Int, y : Int }

type alias AgentModule =
    { agent_id : Maybe String
    , module_type : String
    , position : GridCell
    }

type alias Path =
    { status : Int
    , cost : Float
    , path : List AgentModule
    }

type Msg
    = NoOp
    | ToggleMode
    | ModeChanged (Result Http.Error ())
    | UpdateAgents Decode.Value
    | SetConnected Bool
    | HandleGridClick GridCell
    | CloseMenu
    | StartAgent String GridCell
    | SetPathStart GridCell
    | SetPathGoal GridCell
    | RemoveAgent GridCell
    | StartPlanning Bool
    | PlanningResult (Result Http.Error Path)