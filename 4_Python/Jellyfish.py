# import folium

# # Créez une carte centrée sur les coordonnées GPS de votre choix
# ma_carte = folium.Map(location=[48.853, 2.35], zoom_start=13)

# # Ajoutez des marqueurs pour chaque point GPS que vous souhaitez afficher
# folium.Marker(location=[48.853, 2.35]).add_to(ma_carte)
# folium.Marker(location=[48.856, 2.352]).add_to(ma_carte)
# folium.Marker(location=[48.85, 2.35]).add_to(ma_carte)

# # Ajoutez une couche d'image d'OpenStreetMap pour l'arrière-plan avec attribution
# folium.TileLayer('https://a.tile.openstreetmap.org/{z}/{x}/{y}.png', name='OpenStreetMap', attr='Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors').add_to(ma_carte)

# # Affichez la carte
# ma_carte


# import pandas as pd
# import folium

# # Extraire les colonnes de latitude et de longitude
# latitudes = data['GPS_Latitude']
# longitudes = data['GPS_Longitude']

# # Créer une carte centrée sur les coordonnées moyennes des points
# m = folium.Map(location=[latitudes.mean(), longitudes.mean()], zoom_start=10)

# # Ajouter un marqueur pour chaque point
# for lat, lon in zip(latitudes, longitudes):
#     folium.Marker(location=[lat, lon]).add_to(m)

# # Afficher la carte
# m



# import folium

# # Créez une carte centrée sur les coordonnées GPS de votre choix
# ma_carte = folium.Map(location=[48.853, 2.35], zoom_start=13)

# # Ajoutez des marqueurs pour chaque point GPS que vous souhaitez afficher
# folium.Marker(location=[48.853, 2.35]).add_to(ma_carte)
# folium.Marker(location=[48.856, 2.352]).add_to(ma_carte)
# folium.Marker(location=[48.85, 2.35]).add_to(ma_carte)

# # Ajoutez une couche d'image d'OpenStreetMap pour l'arrière-plan avec attribution
# folium.TileLayer('https://a.tile.openstreetmap.org/{z}/{x}/{y}.png', name='OpenStreetMap', attr='Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors').add_to(ma_carte)

# # Affichez la carte
# ma_carte



import plotly.express as px
df = px.data.gapminder().query("year == 2007")
fig = px.scatter_geo(df, locations="iso_alpha",
                     size="pop", # size of markers, "pop" is one of the columns of gapminder
                     )
fig.show()