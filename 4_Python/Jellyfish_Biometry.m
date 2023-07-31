%% LIRMM : Jellyfish - Biometry
%  Autor : Andrea  Sauviat
%  Date  : 02.07.2022

clear all ; % Clear the Workspace
close all ; % Close of the figure
clc       ; % Clear the Command Window

%%

Jellyfish = [   "20/06/2023" , 28.5 ; "20/06/2023" , 22.5 ; "20/06/2023" , 36.5 ; "20/06/2023" , 25.0 ; "20/06/2023" , 32.0 ; ...
                "20/06/2023" , 26.0 ; "20/06/2023" ,  9.0 ; "20/06/2023" , 20.0 ; "20/06/2023" , 29.5 ; "20/06/2023" , 27.0 ; ...
                "20/06/2023" , 35.0 ; "20/06/2023" , 24.0 ; "20/06/2023" , 10.0 ; "20/06/2023" ,  9.3 ; "20/06/2023" ,  7.3 ; ...
                "20/06/2023" ,  6.5 ; "20/06/2023" , 19.0 ; "20/06/2023" , 28.0 ; "20/06/2023" , 29.0 ; "20/06/2023" , 28.0 ; ...
                "20/06/2023" , 24.5 ; "20/06/2023" , 11.2 ; "20/06/2023" , 11.0 ; "20/06/2023" , 29.5 ; "20/06/2023" , 21.0 ; ...
                "20/06/2023" , 34.5 ; "20/06/2023" , 33.5 ; "20/06/2023" , 27.0 ; "20/06/2023" , 26.0 ; "20/06/2023" , 10.5 ; ...
                "20/06/2023" , 24.3 ; "20/06/2023" , 23.0 ; "20/06/2023" , 25.0 ; "20/06/2023" , 25.0 ; "20/06/2023" ,  7.3 ; ...
                "20/06/2023" , 28.5 ; "20/06/2023" , 22.5 ; "20/06/2023" ,  9.5 ; "20/06/2023" ,  8.5 ; "20/06/2023" , 11.5 ; ...
                "20/06/2023" , 14.0 ; "20/06/2023" ,  7.5 ; "20/06/2023" ,  7.5 ; "20/06/2023" , 10.0 ; "20/06/2023" , 22.0 ; ...
                "20/06/2023" ,  7.3 ; "20/06/2023" , 26.0 ; "20/06/2023" , 35.0 ; "20/06/2023" , 24.0 ; "20/06/2023" , 20.0 ; ...

                "21/06/2023" ,  9.5 ; "21/06/2023" , 11.5 ; "21/06/2023" , 12.5 ; "21/06/2023" , 13.5 ; "21/06/2023" , 12.5 ; ...
                "21/06/2023" , 13.0 ; "21/06/2023" , 15.0 ; "21/06/2023" , 31.0 ; "21/06/2023" , 15.5 ; "21/06/2023" , 13.5 ; ...
                "21/06/2023" , 10.5 ; "21/06/2023" , 11.0 ; "21/06/2023" , 13.5 ...
                
                ] ;


%% Calculate day difference 

% Convert date into 'datetime' type
dates = datetime(Jellyfish(:, 1), 'InputFormat', 'dd/MM/yyyy');

% Search for the most recent date
max_date = max(dates);

for i = 1 : 1 : height( Jellyfish )                                             % For every line in the table ...
    Jellyfish_Date = datetime(Jellyfish(i, 1), 'InputFormat', 'dd/MM/yyyy') ;       % ... Get the date
    Jellyfish( i , 3 ) = - days( max_date - Jellyfish_Date ) ;                        % ... Deduce the day difference
end

clear dates max_date i Jellyfish_Date 


%% Swap row 2 with row 3 for logic purpose

% Swap row 2 with row 3 for logic purpose
Swap_Row = Jellyfish(:,2) ;
Jellyfish(:,2) = Jellyfish(:,3);
Jellyfish(:,3) = Swap_Row ;

clear Swap_Row 


%% Calculate weight of the Jellyfish

for i = 1 : 1 : height( Jellyfish )
    Jellyfish(i,4) = round( 0.19 * power( str2double(Jellyfish(i,3)), 2.63) , 2 ) ;
end

clear i


%%

figure( 1 )
    plot( datetime(Jellyfish(:, 1), 'InputFormat', 'dd/MM/yyyy'), str2double(Jellyfish(:,4)), 'o' ) ;
    title( "Jellyfish biometry") ;
    xlabel( "Time elapsed" ) ;
    xlim( [datetime("19-Jun-2023") datetime("26-Jun-2023")]);
    ylabel( "Jellyfish weight (g)" );
    grid on



