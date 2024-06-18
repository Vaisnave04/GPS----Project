
from flask import Flask, render_template, request
import folium
from geopy.geocoders import Nominatim
from geopy.distance import geodesic
import osmnx as ox
import networkx as nx
import time

app = Flask(__name__, template_folder='templates')

# Declare a global variable for distance
global_distance = 0.0

@app.route('/')
def home():
    return render_template('main.html')





def geocode_location(geolocator, location, max_retries=5, initial_delay=1):
    retries = 0
    delay = initial_delay
    while retries < max_retries:
        try:
            return geolocator.geocode(location, timeout=10)
        except Exception as e:
            print(f"Geocoding error: {e}. Retrying in {delay} seconds...")
            time.sleep(delay)
            delay *= 2
            retries += 1
   
def calculate_distance(origin, destination):
    global global_distance
    geolocator = Nominatim(user_agent="geoapi")

    origin_location = geocode_location(geolocator, origin)
    if not origin_location:
        print(f"Could not geocode origin location: {origin}")
        return None

    destination_location = geocode_location(geolocator, destination)
    if not destination_location:
        print(f"Could not geocode destination location: {destination}")
        return None

    origin_coords = (origin_location.latitude, origin_location.longitude)
    destination_coords = (destination_location.latitude, destination_location.longitude)

    global_distance = geodesic(origin_coords, destination_coords).kilometers
    print("global_distance==",global_distance)
    return global_distance

@app.route('/show_directions', methods=['POST'])
def show_directions():
    
    origin = request.form['origin']
    destination = request.form['destination']

    geolocator1 = Nominatim(user_agent="geoapi")
    origin_location = geolocator1.geocode(origin)
    destination_location = geolocator1.geocode(destination)
    
    if not origin_location or not destination_location:
        return render_template('error.html', message="One of the locations could not be found.")

    
    origin_coords = (origin_location.latitude, origin_location.longitude)
    destination_coords = (destination_location.latitude, destination_location.longitude)
    
    print("origin",origin)
    print("destination",destination)
    distance = calculate_distance(origin, destination)

    print("distance",distance)
    # Get the graph for the area around the origin and destination
    G = ox.graph_from_point(origin_coords, dist=distance, network_type='drive')

    origin_node = ox.distance.nearest_nodes(G, origin_coords[1], origin_coords[0])
    destination_node = ox.distance.nearest_nodes(G, destination_coords[1], destination_coords[0])

    # Calculate the shortest path by distance
    route = nx.shortest_path(G, origin_node, destination_node, weight='length')

    # Create map
    m = folium.Map(location=origin_coords, zoom_start=13)
    folium.Marker(origin_coords, popup=f'Origin: {origin}', icon=folium.Icon(color='green')).add_to(m)
    folium.Marker(destination_coords, popup=f'Destination: {destination}', icon=folium.Icon(color='red')).add_to(m)

    # Plot the route on the map
    route_coordinates = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]
    folium.PolyLine(locations=route_coordinates, color='blue', weight=5, opacity=0.8).add_to(m)
    
    map_html = m._repr_html_()

    return render_template('result.html', map_html=map_html, origin=origin, destination=destination)

if __name__ == '__main__':
     app.run(debug=True)
