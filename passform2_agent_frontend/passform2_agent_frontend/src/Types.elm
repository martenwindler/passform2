module Types exposing (..)

import Dict exposing (Dict)
import Http
import Json.Decode as Decode

-- --- DOMAIN TYPES ---

{-| Repräsentiert die verschiedenen Hardware- und Logik-Module im Gitter. -}
type ModuleType
    = FTF           
    | Conveyeur     
    | RollenModul   
    | Mensch        
    | Greifer       
    | Station       
    | UnknownModule String

{-| Semantische Zustände für das Logging und die UI-Farben. -}
type LogLevel
    = Success       
    | Info          
    | Warning       -- Hier bleibt der Name für die Logs erhalten
    | Danger        

{-| Status der physischen Hardware-Komponenten. 
Umbenannt von Warning zu Standby, um den Elm Name-Clash zu vermeiden.
-}
type HardwareStatus
    = Online
    | Standby  -- Nutze dies für den gelben Status (z.B. im Simulations-Modus)
    | Missing
    | Error
    | UnknownStatus

type Mode 
    = Simulation 
    | Hardware

type SidebarTab
    = TabPlanning    
    | TabAgents      
    | TabHardware    
    | TabLogs        


-- --- ENTITIES ---

type alias GridCell = 
    { x : Int, y : Int }

type alias SystemLog =
    { message : String
    , level : LogLevel
    }

type alias HardwareDevice =
    { pi_id : String
    , rfid_status : HardwareStatus
    , pi_exists : Bool
    }

type alias AgentModule =
    { agent_id : Maybe String
    , module_type : ModuleType
    , position : GridCell
    , orientation : Int
    , is_dynamic : Bool
    , payload : Maybe String
    , signal_strength : Int 
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


-- --- MESSAGES ---

type Msg
    = NoOp
    | ToggleMode
    | ToggleViewMode
    | ToggleSidebar
    | SwitchSidebarTab SidebarTab
    | SetGridWidth String
    | SetGridHeight String
    | CloseMenu
    | SetCurrentAsDefault
    | LoadDefaultConfig
    | ClearGrid
    | SetMode String 
    | ExportConfig
    | ImportConfigTrigger
    | ConfigReceived String
    | StartAgent ModuleType GridCell 
    | RemoveAgent GridCell
    | RotateAgent GridCell
    | MoveAgent { oldX : Int, oldY : Int, newX : Int, newY : Int }
    | UpdateAgents Decode.Value
    | SetConnected Bool 
    | SetRosConnected Bool 
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