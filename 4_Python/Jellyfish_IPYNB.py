# %% [markdown]
# # Introduction to the JupyterLab and Jupyter Notebooks
# 
# This is a short introduction to two of the flagship tools created by [the Jupyter Community](https://jupyter.org).
# 
# > **⚠️Experimental!⚠️**: This is an experimental interface provided by the [JupyterLite project](https://jupyterlite.readthedocs.io/en/latest/). It embeds an entire JupyterLab interface, with many popular packages for scientific computing, in your browser. There may be minor differences in behavior between JupyterLite and the JupyterLab you install locally. You may also encounter some bugs or unexpected behavior. To report any issues, or to get involved with the JupyterLite project, see [the JupyterLite repository](https://github.com/jupyterlite/jupyterlite/issues?q=is%3Aissue+is%3Aopen+sort%3Aupdated-desc).
# 
# ## JupyterLab 🧪
# 
# **JupyterLab** is a next-generation web-based user interface for Project Jupyter. It enables you to work with documents and activities such as Jupyter notebooks, text editors, terminals, and custom components in a flexible, integrated, and extensible manner. It is the interface that you're looking at right now.
# 
# **For an overview of the JupyterLab interface**, see the **JupyterLab Welcome Tour** on this page, by going to `Help -> Welcome Tour` and following the prompts.
# 
# > **See Also**: For a more in-depth tour of JupyterLab with a full environment that runs in the cloud, see [the JupyterLab introduction on Binder](https://mybinder.org/v2/gh/jupyterlab/jupyterlab-demo/HEAD?urlpath=lab/tree/demo).
# 
# ## Jupyter Notebooks 📓
# 
# **Jupyter Notebooks** are a community standard for communicating and performing interactive computing. They are a document that blends computations, outputs, explanatory text, mathematics, images, and rich media representations of objects.
# 
# JupyterLab is one interface used to create and interact with Jupyter Notebooks.
# 
# **For an overview of Jupyter Notebooks**, see the **JupyterLab Welcome Tour** on this page, by going to `Help -> Notebook Tour` and following the prompts.
# 
# > **See Also**: For a more in-depth tour of Jupyter Notebooks and the Classic Jupyter Notebook interface, see [the Jupyter Notebook IPython tutorial on Binder](https://mybinder.org/v2/gh/ipython/ipython-in-depth/HEAD?urlpath=tree/binder/Index.ipynb).
# 
# ## An example: visualizing data in the notebook ✨
# 
# Below is an example of a code cell. We'll visualize some simple data using two popular packages in Python. We'll use [NumPy](https://numpy.org/) to create some random data, and [Matplotlib](https://matplotlib.org) to visualize it.
# 
# Note how the code and the results of running the code are bundled together.

# %%
import matplotlib.pyplot as plt
import numpy as np

# %%
# Open File
with open('Files_txt/Release_01_210423/GNAT_03.txt', 'r') as f:
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

# Affichage des données
for row in parsed_data:
    print(row['GPS_Latitude'])
    
print( parsed_data[9]['Board_Name'])


# %%
import fonctions

Release_01_210423_GNAT_03 = fonctions.Read_File_txt( 'Files_txt/Release_01_210423/GNAT_03.txt' )
print( f"In 'Files_txt/Release_01_210423/GNAT_03.txt', there is {len(Release_01_210423_GNAT_03)+1} data available.")
Release_01_210423_GNAT_04 = fonctions.Read_File_txt( 'Files_txt/Release_01_210423/GNAT_04.txt' )
print( f"In 'Files_txt/Release_01_210423/GNAT_04.txt', there is {len(Release_01_210423_GNAT_04)+1} data available.")



# %%
with open('Files_txt/Release_01_210423/GNAT_03.txt', 'r') as f:
    data = f.read().splitlines()  # lire le fichier et stocker chaque ligne dans un tableau

header = data[0].split(';')  # stocker les noms de colonnes dans un tableau

tableau = []
for ligne in data[1:]:
    colonnes = ligne.split(';')
    tableau.append(colonnes)

# créer un dictionnaire pour stocker chaque colonne sous forme de tableau
tableau_dict = {}
for i, nom_colonne in enumerate(header):
    tableau_dict[nom_colonne] = [ligne[i] for ligne in tableau]

# afficher chaque colonne
for nom_colonne, colonne in tableau_dict.items():
    print(nom_colonne, colonne)

# %%
import pandas as pd

column_names = ['Board_Name', 'Battery_Level', 'Time_User', 'Time_PreviousMsg', 'Time_Elapsed', 'time', 'STM32_Temperature', 
                'DevEUI', 'Network_Quality', 'RSSI', 'SNR', 'ESP', 'SF', 'Frequency', 'Nb_Gateways', 'fcnt', 'LoRa_Payload', 
                'GPS_Latitude', 'GPS_Longitude', 'GPS_NbSatellites', 'GPS_EHPE', 'GPS_Distance', 'GPS_Vitesse', 'GPS_Direction', 
                'Acc_Temp', 'Acc_AxeX', 'Acc_AxeY', 'Acc_AxeZ', 'Acc_Roll', 'Acc_Pitch', 'Acc_Yaw']

# Lire le fichier CSV et créer une table de données
data = pd.read_csv('Files_txt/Release_01_210423/GNAT_03.txt', sep=";", names=column_names)

# Afficher la table de données
print(data['GPS_Latitude'])

GPS = data.loc[:,['GPS_Latitude', 'GPS_Longitude']]
print( GPS )

plt.plot( data['GPS_Latitude'], data['GPS_Longitude'])

# %%
import pandas as pd
import folium

# Extraire les colonnes de latitude et de longitude
latitudes = data['GPS_Latitude']
longitudes = data['GPS_Longitude']

# Créer une carte centrée sur les coordonnées moyennes des points
m = folium.Map(location=[latitudes.mean(), longitudes.mean()], zoom_start=10)

# Ajouter un marqueur pour chaque point
for lat, lon in zip(latitudes, longitudes):
    folium.Marker(location=[lat, lon]).add_to(m)

# Afficher la carte
m

# %%
from mpl_toolkits.basemap import Basemap



