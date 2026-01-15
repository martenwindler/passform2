module Types exposing (..)

import Dict exposing (Dict)
import Http
import Json.Decode as Decode

-- --- MODEL ---

type alias Model =
    { mode : Mode
    , backendIP : String
    , connected : Bool
    , agents : Dict (Int, Int) AgentModule
    , savedDefault : Dict (Int, Int) AgentModule
    , logs : List String             -- Liste der System-Nachrichten
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

-- --- ENTITIES ---

type alias GridCell = 
    { x : Int, y : Int }

type alias AgentModule =
    { agent_id : Maybe String
    , module_type : String
    , position : GridCell
    , orientation : Int             -- 0, 90, 180, 270 Grad
    }

type alias Path =
    { status : Int
    , cost : Float
    , path : List AgentModule
    }

-- --- MESSAGES ---

type Msg
    = NoOp
    -- Navigation & UI
    | ToggleMode
    | ToggleViewMode
    | ToggleSidebar
    | SetGridWidth String
    | SetGridHeight String
    | CloseMenu
    -- Konfiguration & Persistenz
    | SetCurrentAsDefault           -- Speichert aktuelles Layout im LocalStorage
    | LoadDefaultConfig             -- Lädt das gespeicherte Layout
    | ClearGrid
    | ExportConfig                  -- JSON Datei-Export
    | ImportConfigTrigger           -- Öffnet Datei-Dialog
    | ConfigReceived String         -- Verarbeitet importiertes JSON
    -- Agenten-Management
    | StartAgent String GridCell    -- Platziert neuen Agenten
    | RemoveAgent GridCell          -- Löscht Agenten
    | RotateAgent GridCell          -- Dreht Modul um 90 Grad
    | MoveAgent { oldX : Int, oldY : Int, newX : Int, newY : Int }
    -- Backend Kommunikation
    | UpdateAgents Decode.Value      -- Live-Update vom Socket
    | SetConnected Bool             -- Socket Verbindungsstatus
    | LogReceived String            -- Nachricht vom Backend-Log
    | HandleGridClick GridCell      -- Klick-Event aus der 3D-Szene
    -- Pfadplanung
    | SetPathStart GridCell
    | SetPathGoal GridCell
    | StartPlanning Bool
    | PlanningResult (Result Http.Error Path)
    | PlanningResultRaw Decode.Value
    | ModeChanged (Result Http.Error ())