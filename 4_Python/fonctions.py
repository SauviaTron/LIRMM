def Read_File_txt( File_Name ):
    """_summary_

    Args:
        File_Name (string): Path and name of your file. For example : 'Files_txt/Release_XX_DDMMYY/GNAT_YY.txt'
    """

    # Open File
    with open(File_Name, 'r') as f:
        data = f.readlines()

    column_names = ['Board_Name', 'Battery_Level', 'Time_User', 'Time_PreviousMsg', 'Time_Elapsed', 'time', 'STM32_Temperature', 
                    'DevEUI', 'Network_Quality', 'RSSI', 'SNR', 'ESP', 'SF', 'Frequency', 'Nb_Gateways', 'fcnt', 'LoRa_Payload', 
                    'GPS_Latitude', 'GPS_Longitude', 'GPS_NbSatellites', 'GPS_EHPE', 'GPS_Distance', 'GPS_Vitesse', 'GPS_Direction', 
                    'Acc_Temp', 'Acc_AxeX', 'Acc_AxeY', 'Acc_AxeZ', 'Acc_Roll', 'Acc_Pitch', 'Acc_Yaw']

    # Suppression des caractères spéciaux des noms de colonnes
    column_names = [name.replace('\n', '').replace(' ', '_').replace('-', '_').replace('.', '_') for name in column_names]

    # Suppression de la première ligne qui contient les noms de colonnes
    data = data[1:]

    # Création d'une liste vide pour stocker les données
    parsed_data = []

    # Traitement des données et stockage dans la liste parsed_data
    for line in data:
        row = line.strip().split(';')
        parsed_row = {}
        for i in range(len(row)):
            parsed_row[column_names[i]] = row[i]
        parsed_data.append(parsed_row)

    return parsed_data








# # Time_Release_Start = '23.04.21 - 9h00'
# # Time_Release_End   = '23.04.21 - 10h00'


# def Split_TimeUser( Time_User ):
    
#     Date_YYMMDD, Date_HHMM = Time_User.split(' - ')
#     Date_Year, Date_Month, Date_Day = Date_YYMMDD.split('.')
#     Date_Hour, Date_Minute = Date_HHMM.split('h')
        
#     return Date_Year, Date_Month, Date_Day, Date_Hour, Date_Minute
    
# def Get_First_Line(  ):
#     # Start_Year, Start_Month, Start_Day, Start_Hour, Start_Minute = Split_TimeUser( Time_Release_Start )
#     # print(Start_Year, Start_Month, Start_Day, Start_Hour, Start_Minute)
#     for i in range( len(Data) ):
#         Check_Year, Check_Month, Check_Day, Check_Hour, Check_Minute = Split_TimeUser( Data[i]['Time_User'] )
#         if( (Check_Year>='23') and (Check_Month>='04') and (Check_Day>='21') and (Check_Hour>'11') and (Check_Minute>='00')  ):
#             print( f"First ligne: {i} \t {Data[i]['Time_User']}" )
#             return i
        
# def Get_Last_Line( Time_Release_Stop ):
#     Stop_Year, Stop_Month, Stop_Day, Stop_Hour, Stop_Minute = Split_TimeUser( Time_Release_Stop )
#     print(Stop_Year, Stop_Month, Stop_Day, Stop_Hour, Stop_Minute)
#     for i in range( len(Data) ):
#         Check_Year, Check_Month, Check_Day, Check_Hour, Check_Minute = Split_TimeUser( Data[i]['Time_User'] )
#         if( (Check_Year>=Stop_Year) and (Check_Month>=Stop_Month) and (Check_Day>=Stop_Day) and (Check_Hour>=Stop_Hour) and (Check_Minute<='59')  ):
#             print( f"Last ligne: {i} \t {Data[i]['Time_User']} ")
#             return i
        
# def Get_ReleaseStartStop( ):
    
#     Release_Start = Get_First_Line( )
#     Release_Stop = Get_Last_Line( '23.04.21 - 9h00')
#     return Release_Start, Release_Stop

# Release_Start, Release_Stop = Get_ReleaseStartStop( ) 

# print( Release_Start )
# print( Release_Stop )
    