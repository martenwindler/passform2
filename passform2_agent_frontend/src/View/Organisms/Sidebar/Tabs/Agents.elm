module View.Organisms.Sidebar.Tabs.Agents exposing (view, formatModuleType)

import Html exposing (..)
import Html.Attributes exposing (..)
import Types exposing (..)
import Types.Domain exposing (..) -- Das hier hat gefehlt!
import View.Atoms.Button as Button
import View.Organisms.Sidebar.Sections.ActiveModuleSection as ActiveModuleSection

{-|
Der Agents-Tab fungiert als Orchestrator. 
-}

view : Model -> Html Msg
view model =
    div [ class "sidebar-tab-content tab-agents h-full flex flex-col" ]
        [ ActiveModuleSection.view model
        , div [ class "mt-auto pt-4 border-t border-white/5" ]
            [ Button.danger 
                "Gesamtes Gitter leeren" 
                (AgentsMsg ClearGrid) 
                False -- isPrimary
                True  -- isDisabled (Hier auf True setzen!)
            ]
        ]

{-| 
PROXY-FUNKTION FÜR TESTS:
Wir holen uns die Logik aus der ActiveModuleSection.
-}
formatModuleType : ModuleType -> String
formatModuleType mType =
    ActiveModuleSection.formatModuleType mType