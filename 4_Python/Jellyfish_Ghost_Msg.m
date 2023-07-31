%% LIRMM : Jellyfish - Biometry
%  Autor : Andrea  Sauviat
%  Date  : 02.07.2022

clear all ; % Clear the Workspace
close all ; % Close of the figure
clc       ; % Clear the Command Window

%%

Date = "07_100723" ;
File = "GNAT_02.txt" ;

Orange_Live_Object_File = readtable( "Ghost_Msg\" + Date + "\Orange\" + File );
Node_RED_File = readtable("Ghost_Msg\" + Date + "\Node-RED\" + File);


%%


Messages_Received_by_Orange = Orange_Live_Object_File.fcnt(:) ;
Messages_Receivec_by_NodeRED = Node_RED_File.Var16(:) ;


elements_non_communs = setdiff(Messages_Received_by_Orange, Messages_Receivec_by_NodeRED );

Nb_GhostMsg = 0 ; 

for i = 1 : 1 : height( elements_non_communs )
    for j = 1 : 1 : height( Messages_Received_by_Orange )
        if( Orange_Live_Object_File.fcnt(j) == elements_non_communs(i) )
            fprintf( "Fcnt : %d \t ", Orange_Live_Object_File.fcnt(j) ) ;
            cellule = Orange_Live_Object_File.date(j) ;
            date_str = extractBetween(cellule{1}, 1, 10);
            time_str = extractBetween(cellule{1}, 12, 19);
            str = [date_str{1} ' ' time_str{1}];
            disp(str);
            Nb_GhostMsg = Nb_GhostMsg + 1 ;
        end
    end
end

fprintf("Total : %d \n", Nb_GhostMsg ) ;






