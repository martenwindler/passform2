module ViewHelperTest exposing (suite)

import Test exposing (Test, describe, test)
import Expect
import Types.Domain exposing (..)
import View.Organisms.Sidebar.Tabs.Agents as AgentsTab

suite : Test
suite =
    describe "View Helper Tests"
        [ test "Formatierung von Modul-Typen" <|
            \_ ->
                AgentsTab.formatModuleType FTF 
                    |> Expect.equal "FTF"

        , test "Formatierung unbekannter Module" <|
            \_ ->
                AgentsTab.formatModuleType (UnknownModule "Scanner") 
                    |> Expect.equal "Unbekannt: Scanner"
        ]