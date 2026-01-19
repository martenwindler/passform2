module View.Organisms.Sidebar.Sections.FileConfigSection exposing (view)

import Html exposing (..)
import Html.Attributes exposing (..)
import Html.Events exposing (onClick)
import Types exposing (..)

{-| 
Diese Sektion kümmert sich um das Laden, Speichern und Exportieren 
von Gitter-Konfigurationen (JSON/Default).
-}
view : Model -> Html Msg
view model =
    div [ class "planning-section system-management mt-8 pt-4 border-t border-white/5" ]
        [ h4 [] [ text "System-Management" ]
        
        , -- Grid für die Datei-Operationen
          div [ class "grid grid-cols-2 gap-2" ]
            [ button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem]"
                , onClick (AgentsMsg LoadDefaultConfig) 
                ] 
                [ text "📂 Default" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem]"
                , onClick (AgentsMsg SetCurrentAsDefault) 
                ] 
                [ text "💾 Save" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem]"
                , onClick (AgentsMsg ExportConfig) 
                ] 
                [ text "📤 Export" ]
            
            , button 
                [ class "btn-secondary btn-full py-2 text-[0.7rem]"
                , onClick (AgentsMsg ImportConfigTrigger) 
                ] 
                [ text "📥 Import" ]
            ]
            
        , -- Destruktive Aktion: Gitter leeren
          button 
            [ class "btn-danger btn-full mt-2 text-[0.75rem] py-2"
            , onClick (AgentsMsg ClearGrid) 
            ] 
            [ text "🗑️ Gitter leeren" ]
        ]