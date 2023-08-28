%% LIRMM  : Simulation of a consumption
%  Autor : Andrea  Sauviat
%  Date   : 04.05.2022

% Signal Processing Toolbox
% Communications Toolbox 

clear all ;
close all ;
clc ;


Battery_Capacity = 500e-3                  ; % mAh - Capacity of one battery
Battery_Capacity = Battery_Capacity * 1     ; % mAh - Capacity of the total battery
Battery_Capacity = Battery_Capacity * 0.9; 

Time_Running = 60 ; % [s]
Every_X_Time = 5 ; % [min]

Add_Noise = false ;

GNAT_32MHz_Run    = 40e-3   ; % [A]
GNAT_32MHz_Sleep  = 1.65e-6 ; % [A]
GNAT_16MHz_Run    = 32e-3   ; % [A]
GNAT_16MHz_Sleep  = .783e-6 ; % [A]
GNAT_4_2MHz_Run   = 30e-3   ; % [A]
GNAT_4_2MHz_Sleep = .181e-6 ; % [A]

[ GNAT_32MHz , t , GNAT_32MHz_Hour_Capacity , GNAT_32MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
    GNAT_32MHz_Run , GNAT_32MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
[ GNAT_16MHz , t , GNAT_16MHz_Hour_Capacity , GNAT_16MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
    GNAT_16MHz_Run , GNAT_16MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
[ GNAT_4_2MHz , t , GNAT_4_2MHz_Hour_Capacity , GNAT_4_2MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
    GNAT_4_2MHz_Run , GNAT_4_2MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 


%% Results

clc

fprintf("GNAT-L082CZ \t Hour         \t Day       \n");
fprintf("----------- \t -----------  \t --------- \n");
fprintf("@ 32MHz     \t %.2f         \t %.2f \n", GNAT_32MHz_Hour_Capacity , GNAT_32MHz_Day_Capacity);
fprintf("@ 16MHz     \t %.2f         \t %.2f \n", GNAT_16MHz_Hour_Capacity , GNAT_16MHz_Day_Capacity);
fprintf("@ 4.2MHz    \t %.2f         \t %.2f \n", GNAT_4_2MHz_Hour_Capacity , GNAT_4_2MHz_Day_Capacity);


%%
figure( 1 ) ,
    
    subplot(211),
    hold on
    a = area(t,GNAT_32MHz * 1000);
    b = area(t,GNAT_16MHz * 1000);
    c = area(t,GNAT_4_2MHz * 1000);
    hold off
    grid on
    %axis([0 120 0 45]);
    xlabel("Time (min)");
    ylabel("Amplitude (mA)");
    title("Fig.1 - Simulation de la consommation");
    legend("32MHz" , "16MHz" , "4.2MHz");


    subplot(212),
    hold on
    e = area(t,GNAT_32MHz * 1000);
    f = area(t,GNAT_16MHz * 1000);
    g = area(t,GNAT_4_2MHz * 1000);
    hold off
    axis([0 1.5 0 45]);
    grid on
	xlabel("Time (min)");
    ylabel("Amplitude (mA)");
    title("Fig.2 - Zoom [0 - 1min]");
    legend("32MHz" , "16MHz" , "4.2MHz");

return
    %%

[GNAT_MHz_Hour_Capacity_TAB GNAT_MHz_Day_Capacity_TAB] = Variation_Every_X_Time( 1,15 ) ;

%%
figure, 
    sgtitle("Evolution du la durée de déploiement en fonction de la fréquence d'émission");
    subplot(121)
    hold on
    scatter(GNAT_MHz_Hour_Capacity_TAB(:,1),GNAT_MHz_Hour_Capacity_TAB(:,2),"filled", SizeData=100);
    scatter(GNAT_MHz_Hour_Capacity_TAB(:,1),GNAT_MHz_Hour_Capacity_TAB(:,3),"filled", SizeData=100);
    scatter(GNAT_MHz_Hour_Capacity_TAB(:,1),GNAT_MHz_Hour_Capacity_TAB(:,4),"filled", SizeData=100 );
    hold off
    grid on
    xlabel("Time (min)");
    ylabel("Durée de vie (h)");
    %title("Evolution du durée de déploiement en fonction de la fréquence d'émission");
    legend("32MHz" , "16MHz" , "4.2MHz", Location="northwest");
    
    subplot(122)
    hold on
    scatter(GNAT_MHz_Day_Capacity_TAB(:,1),GNAT_MHz_Day_Capacity_TAB(:,2),"filled", SizeData=100);
    scatter(GNAT_MHz_Day_Capacity_TAB(:,1),GNAT_MHz_Day_Capacity_TAB(:,3),"filled", SizeData=100);
    scatter(GNAT_MHz_Day_Capacity_TAB(:,1),GNAT_MHz_Day_Capacity_TAB(:,4),"filled", SizeData=100 );
    hold off
    grid on
    xlabel("Time (min)");
    ylabel("Durée de vie (j)");
    %title("Evolution du durée de déploiement en fonction de la fréquence d'émission");
    legend("32MHz" , "16MHz" , "4.2MHz", Location="northwest");

return
%%

[GNAT_MHz_Hour_Capacity_TAB GNAT_MHz_Day_Capacity_TAB] = Variation_EveryXTime_AND_TimeRunning( 1,15,30,60 ) ;

%%

[xq,yq] = meshgrid(1:1:15, 30:5:60);

fig = figure;
Plot_Scatter_1 = scatter3( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,3) , ...
    "filled", SizeData=100) ;
hold on
Plot_Scatter_2 = scatter3( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,4) , ...
    "filled", SizeData=100) ;
hold on
Plot_Scatter_3 = scatter3( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,5) , ...
    "filled", SizeData=100) ;
% hold on
% vq = griddata( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,3),xq,yq )
% surf(xq,yq,vq,'FaceAlpha',0.5,EdgeColor = 'none',FaceColor=	"#0072BD")
% hold on
% vq = griddata( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,4),xq,yq )
% surf(xq,yq,vq,'FaceAlpha',0.5,EdgeColor = 'none',FaceColor=	"#D95319")
% hold on
% vq = griddata( GNAT_MHz_Hour_Capacity_TAB(:,1) , GNAT_MHz_Hour_Capacity_TAB(:,2) , GNAT_MHz_Day_Capacity_TAB(:,5),xq,yq )
% surf(xq,yq,vq,'FaceAlpha',0.5,EdgeColor = 'none',FaceColor="#EDB120")
hold off
    %xlabel( "Fréquence d'émission (min)" ) ; xtickangle(45)  ;
    %ylabel( "Durée de la recherche GPS (sec)" ) ; ytickangle(45) ;
    zlabel( "Durée de vie (j)" ) ;
    title("Evolution du durée de déploiement en fonction de la fréquence d'émission et de fonctionnement");
    legend("32MHz" , "16MHz" , "4.2MHz" , Location="northwest");
    Plot_Scatter_1.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    Plot_Scatter_1.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    Plot_Scatter_1.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    Plot_Scatter_2.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    Plot_Scatter_2.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    Plot_Scatter_2.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (h)"; 
    Plot_Scatter_3.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    Plot_Scatter_3.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    Plot_Scatter_3.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (h)"; 


%%
clc

x = GNAT_MHz_Day_Capacity_TAB(:,1) ; x2=x ;
y = GNAT_MHz_Day_Capacity_TAB(:,2) ; y2 = y ;
z = GNAT_MHz_Day_Capacity_TAB(:,3) ;
z2 = GNAT_MHz_Day_Capacity_TAB(:,4) ;

% Créer des données de test
x = GNAT_MHz_Day_Capacity_TAB(:,1) ; x2 = x ; x3 = x ;
y = GNAT_MHz_Day_Capacity_TAB(:,2) ; y2 = y ; y3 = y ;
z = GNAT_MHz_Day_Capacity_TAB(:,3) ; z2 = GNAT_MHz_Day_Capacity_TAB(:,4) ; z3 = GNAT_MHz_Day_Capacity_TAB(:,5) ;

% Créer la figure 3D avec scatter3
figure

h1 = scatter3(x,y,z,"filled", SizeData=100);
axis([0 15 30 60 0 12]); % Définir les limites des axes

hold on % Garder la figure existante

% Ajouter la deuxième courbe à la figure avec scatter3
h2 = scatter3(x2,y2,z2,"filled", SizeData=100);

hold on % Garder la figure existante

% Ajouter la deuxième courbe à la figure avec scatter3
h3 = scatter3(x3,y3,z3,"filled", SizeData=100);

% Ajouter des labels aux axes
xlabel( "Fréquence d'émission (min)" ) ; xtickangle(45)  ;
ylabel( "Durée de la recherche GPS (sec)" ) ; ytickangle(45) ;
zlabel( "Durée de vie (j)" ) ;
title("Evolution du durée de déploiement en fonction de la fréquence d'émission et de fonctionnement");

legend("32MHz" , "16MHz" , "4.2MHz" , Location="northeast");

h1.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
h1.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
h1.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
h2.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
h2.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
h2.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
h3.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
h3.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
h3.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 

% Créer une boucle pour faire la rotation
for i = 1:360

    view(i,30); % Définir la vue de la figure en fonction de l'angle i
    drawnow; % Actualiser la figure

    % Capture de la frame courante pour créer le gif
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    % Écrire la première frame avec le paramètre 'gif' pour initialiser le gif
    if i == 1
        imwrite(imind,cm,'rotation.gif','gif','Loopcount',inf);
    % Écrire les frames suivantes en ajoutant 'gif' et 'writemode' à imwrite
    else
        imwrite(imind,cm,'rotation.gif','gif','WriteMode','append');
    end
end


%% Figure

figure, 

    subplot( 2,2,[1,3] )
    h1 = scatter3(x,y,z,"filled", SizeData=100);
    axis([0 15 30 60 0 12]); % Définir les limites des axes
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h2 = scatter3(x2,y2,z2,"filled", SizeData=100);
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h3 = scatter3(x3,y3,z3,"filled", SizeData=100);
    
    % Ajouter des labels aux axes
    xlabel( "Fréquence d'émission (min)" ) ; xtickangle(45)  ;
    ylabel( "Durée de la recherche GPS (sec)" ) ; ytickangle(45) ;
    zlabel( "Durée de vie (j)" ) ;
    %title("Evolution du durée de déploiement en fonction de la fréquence d'émission et de fonctionnement");
    
    legend("32MHz" , "16MHz" , "4.2MHz" , Location="northeast");
    
    h1.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h1.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h1.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h2.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h2.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h2.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h3.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h3.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h3.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 

    subplot( 2,2,2 )
    h1 = scatter3(x,y,z,"filled", SizeData=100);
    axis([0 15 30 60 0 12]); % Définir les limites des axes
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h2 = scatter3(x2,y2,z2,"filled", SizeData=100);
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h3 = scatter3(x3,y3,z3,"filled", SizeData=100);
    
    % Ajouter des labels aux axes
    xlabel( "Fréquence d'émission (min)" ) ; xtickangle(45)  ;
    ylabel( "Durée de la recherche GPS (sec)" ) ; ytickangle(45) ;
    zlabel( "Durée de vie (j)" ) ;
    %title("Evolution du durée de déploiement en fonction de la fréquence d'émission et de fonctionnement");
    
    legend("32MHz" , "16MHz" , "4.2MHz" , Location="northeast");
    
    h1.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h1.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h1.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h2.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h2.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h2.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h3.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h3.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h3.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    
    subplot( 2,2,4 )
    h1 = scatter3(x,y,z,"filled", SizeData=100);
    axis([0 15 30 60 0 12]); % Définir les limites des axes
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h2 = scatter3(x2,y2,z2,"filled", SizeData=100);
    
    hold on % Garder la figure existante
    
    % Ajouter la deuxième courbe à la figure avec scatter3
    h3 = scatter3(x3,y3,z3,"filled", SizeData=100);
    
    % Ajouter des labels aux axes
    xlabel( "Fréquence d'émission (min)" ) ; xtickangle(45)  ;
    ylabel( "Durée de la recherche GPS (sec)" ) ; ytickangle(45) ;
    zlabel( "Durée de vie (j)" ) ;
    %title("Evolution du durée de déploiement en fonction de la fréquence d'émission et de fonctionnement");
    
    legend("32MHz" , "16MHz" , "4.2MHz" , Location="northeast");
    
    h1.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h1.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h1.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h2.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h2.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h2.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 
    h3.DataTipTemplate.DataTipRows(1).Label = "Fréquence d'émission (min)";
    h3.DataTipTemplate.DataTipRows(2).Label = "Durée de la recherche GPS (sec)"; 
    h3.DataTipTemplate.DataTipRows(3).Label = "Durée de vie (j)"; 



%% 

function [ Conso_Plot , t, Hour_Capacity , Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
    Conso_Run , Conso_Sleep , Time_Run , Every_X_Time , Add_Noise )

    Simu_End_Time = 60 * 24 ; % min - Time of simulation
    Resolution = 2e2 ;
    t  = 0 : 1/Resolution : Simu_End_Time ;

    Pulse_Width = Time_Run / 60 ;

    Pulse_Time  = Every_X_Time    ; % min - Every x min, run mode will be enable during IOT_Pulse_Width

    Pulse_Periods = [ 0 : Simu_End_Time ] * Pulse_Time ;
    Conso_Plot = pulstran( t-(Pulse_Width/2) , Pulse_Periods , @rectpuls , Pulse_Width ) ;
    Conso_Plot = Conso_Plot * Conso_Run + Conso_Sleep ;

    if( Add_Noise == true )
        for i = 1 : width(Conso_Plot)
               Conso_Plot(i) = Conso_Plot(i) +  Conso_Plot(i)* ((rand(1) - 0.5)/4) ;
        end
    end

    Hour_Capacity = Battery_Capacity / mean( Conso_Plot ) ;
    
    Day_Capacity = Hour_Capacity / 24 ;

end


function [ GNAT_MHz_Hour_Capacity_TAB GNAT_MHz_Day_Capacity_TAB ] = Variation_Every_X_Time( Every_X_Time_Start , Every_X_Time_Finish )

    Battery_Capacity = 155e-3                  ; % mAh - Capacity of one battery
    Battery_Capacity = Battery_Capacity * 2     ; % mAh - Capacity of the total battery
    Battery_Capacity = Battery_Capacity * 0.9; 
    
    Time_Running = 60 ; % [s]
    
    Add_Noise = false ;
    
    GNAT_32MHz_Run    = 36e-3   ; % [A]
    GNAT_32MHz_Sleep  = 1.65e-6 ; % [A]
    GNAT_16MHz_Run    = 32e-3   ; % [A]
    GNAT_16MHz_Sleep  = .783e-6 ; % [A]
    GNAT_4_2MHz_Run   = 30e-3   ; % [A]
    GNAT_4_2MHz_Sleep = .181e-6 ; % [A]
    
    GNAT_MHz_Hour_Capacity_TAB  = [] ;
    GNAT_MHz_Day_Capacity_TAB   = [] ;
    
    for Every_X_Time = Every_X_Time_Start:1:Every_X_Time_Finish
    
        [ GNAT_32MHz , t , GNAT_32MHz_Hour_Capacity , GNAT_32MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_32MHz_Run , GNAT_32MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ;
    
        [ GNAT_16MHz , t , GNAT_16MHz_Hour_Capacity , GNAT_16MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_16MHz_Run , GNAT_16MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
    
        [ GNAT_4_2MHz , t , GNAT_4_2MHz_Hour_Capacity , GNAT_4_2MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_4_2MHz_Run , GNAT_4_2MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
    
        GNAT_MHz_Hour_Capacity_TAB = [GNAT_MHz_Hour_Capacity_TAB ; Every_X_Time GNAT_32MHz_Hour_Capacity GNAT_16MHz_Hour_Capacity GNAT_4_2MHz_Hour_Capacity]
        GNAT_MHz_Day_Capacity_TAB  = [GNAT_MHz_Day_Capacity_TAB  ; Every_X_Time GNAT_32MHz_Day_Capacity  GNAT_16MHz_Day_Capacity  GNAT_4_2MHz_Day_Capacity] ;
    
    end

end


function [ GNAT_MHz_Hour_Capacity_TAB GNAT_MHz_Day_Capacity_TAB ] = Variation_EveryXTime_AND_TimeRunning( Every_X_Time_Start , Every_X_Time_Finish , Time_Running_Start , Time_Running_Finish)

    Battery_Capacity = 155e-3                  ; % mAh - Capacity of one battery
    Battery_Capacity = Battery_Capacity * 2     ; % mAh - Capacity of the total battery
    Battery_Capacity = Battery_Capacity * 0.9; 
    
    Add_Noise = false ;
    
    GNAT_32MHz_Run    = 36e-3   ; % [A]
    GNAT_32MHz_Sleep  = 1.65e-6 ; % [A]
    GNAT_16MHz_Run    = 32e-3   ; % [A]
    GNAT_16MHz_Sleep  = .783e-6 ; % [A]
    GNAT_4_2MHz_Run   = 30e-3   ; % [A]
    GNAT_4_2MHz_Sleep = .181e-6 ; % [A]
    
    GNAT_MHz_Hour_Capacity_TAB  = [] ;
    GNAT_MHz_Day_Capacity_TAB   = [] ;
    
    for Every_X_Time = Every_X_Time_Start:1:Every_X_Time_Finish
        for Time_Running = Time_Running_Start:5:Time_Running_Finish
    
        [ GNAT_32MHz , t , GNAT_32MHz_Hour_Capacity , GNAT_32MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_32MHz_Run , GNAT_32MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ;
    
        [ GNAT_16MHz , t , GNAT_16MHz_Hour_Capacity , GNAT_16MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_16MHz_Run , GNAT_16MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
    
        [ GNAT_4_2MHz , t , GNAT_4_2MHz_Hour_Capacity , GNAT_4_2MHz_Day_Capacity] = GNATL082CZ( Battery_Capacity , ...
        GNAT_4_2MHz_Run , GNAT_4_2MHz_Sleep , Time_Running , Every_X_Time , Add_Noise ) ; 
    
        GNAT_MHz_Hour_Capacity_TAB = [GNAT_MHz_Hour_Capacity_TAB ; Every_X_Time Time_Running GNAT_32MHz_Hour_Capacity*0.9 GNAT_16MHz_Hour_Capacity*0.9 GNAT_4_2MHz_Hour_Capacity*0.9]
        GNAT_MHz_Day_Capacity_TAB  = [GNAT_MHz_Day_Capacity_TAB  ; Every_X_Time Time_Running GNAT_32MHz_Day_Capacity*0.9  GNAT_16MHz_Day_Capacity*0.9  GNAT_4_2MHz_Day_Capacity*0.9] ;
    
        end
    end

end





