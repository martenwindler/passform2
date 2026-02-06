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

-- KORREKTUR: GridCell ist nun 3D-fähig und enthält das logische Level
type alias GridCell = 
    { x : Int
    , y : Int
    , z : Int      -- Rohwert (z.B. in mm vom Backend)
    , level : Int  -- Ebene (0 = Boden, 1 = Auf Tisch, etc.)
    }

-- Räumliche Punkt-Daten für 3D-Engine und präzise Bay-Positionen
type alias Point =
    { x : Float, y : Float, z : Float }

type alias Quaternion =
    { x : Float, y : Float, z : Float, w : Float }

type alias Pose =
    { position : Point
    , orientation : Quaternion
    }

-- --- BUCHTEN (BAYS) & LAYOUT ---

type alias Bay =
    { unique_id : String
    , name : String
    , origin : Point
    , is_virtual : Bool
    , status : HardwareStatus
    , occupation : Bool
    , module_uuid : String
    , level : Int -- Auch Bays sollten wissen, auf welcher Ebene sie liegen
    }

-- --- INVENTAR & ITEMS ---

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

-- --- SYSTEM-ENTITIES ---

type alias SystemLog =
    { message : String
    , level : LogLevel
    }

type alias HardwareDevice =
    { pi_id : String
    , rfid_status : HardwareStatus
    , pi_exists : Bool
    }

-- KORREKTUR: AgentModule nutzt nun die 3D GridCell
type alias AgentModule =
    { agent_id : Maybe String
    , module_type : ModuleType
    , position : GridCell        -- Enthält x, y, z und level
    , orientation : Int
    , is_dynamic : Bool
    , payload : Maybe String
    , signal_strength : Int 
    , status : HardwareStatus    -- Synchronisiert mit Backend-Zustand
    }

type alias Path =
    { status : Int
    , cost : Float
    , path : List GridCell -- Ein Pfad besteht aus Koordinaten-Punkten
    }

type alias PlanningWeights =
    { execution_time_default : Float
    , complex_module_time : Float
    , human_extra_weight : Float
    , proximity_penalty : Float
    , hardware_safety_factor : Float
    }