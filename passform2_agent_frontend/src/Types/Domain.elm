module Types.Domain exposing (..)

import Dict exposing (Dict)
import Json.Decode as Decode

-- --- BASIS TYPEN ---

type ModuleType
    = FTF           
    | Conveyeur     
    | RollenModul   
    | Mensch         
    | Greifer       
    | Station       
    | UnknownModule String

type LogLevel
    = Success       
    | Info          
    | Warning       
    | Danger        

type HardwareStatus
    = Online
    | Standby  
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
    | TabRanger
    | TabGrid    
    | TabLogs 

-- --- ENTITIES ---

type alias GridCell = 
    { x : Int, y : Int }

-- NEU: Räumliche Punkt-Daten für 3D und Bays
type alias Point =
    { x : Float, y : Float, z : Float }

type alias Quaternion =
    { x : Float, y : Float, z : Float, w : Float }

type alias Pose =
    { position : Point
    , orientation : Quaternion
    }

-- --- NEU: BUCHTEN (BAYS) & LAYOUT ---

type alias Bay =
    { unique_id : String
    , name : String
    , origin : Point
    , is_virtual : Bool
    , status : HardwareStatus
    , occupation : Bool
    , module_uuid : String
    }

-- --- NEU: INVENTAR & ITEMS ---

type alias WorldLocation =
    { frame_id : String
    , pose : Pose
    }

type alias WorldItem =
    { name : String
    , uid : String
    , quantity : Int
    , location : WorldLocation
    }

-- --- BESTANDS-ENTITIES ---

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

type alias PlanningWeights =
    { execution_time_default : Float
    , complex_module_time : Float
    , human_extra_weight : Float
    , proximity_penalty : Float
    , hardware_safety_factor : Float
    }