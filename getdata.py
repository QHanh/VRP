import requests
import numpy as np
import json

url = "https://api.geoapify.com/v1/routematrix?apiKey=ad2c3b02ebaf4ca493709433334288eb"

headers = {"Content-Type": "application/json"}

# read json file
file_path = 'thunghiem2.json'
with open(file_path, 'r', encoding='utf-8') as file:
    json_data = json.load(file)

# data input
if 'time_windows' in json_data:
    time_windows = [(time[0], time[1]) for time_obj in json_data['time_windows'] for time in [time_obj["time"]]]
demands = json_data['demands']
num_vehicles = json_data['num_vehicles']
vehicle_capacities = json_data['vehicle_capacities']
starts = json_data['starts']
ends = json_data['ends']

# data coordinate request
data = {
    'mode': 'drive', 
    'sources': json_data['coordinates'], 
    'targets': json_data['coordinates']
}

# request
resp = requests.post(url, headers=headers, json=data)

# Raise an HTTPError for bad responses
resp.raise_for_status()

# If the request is successful, extract data from the response
result_data = resp.json()

num_sources = len(result_data["sources"])
num_targets = len(result_data["targets"])

# Initialize distance and time matrices with large values
distance_matrix = np.zeros((num_sources, num_targets)).astype(int)
time_matrix = np.zeros((num_sources, num_targets)).astype(int)

# Populate matrices with actual values
for result in result_data["sources_to_targets"]:
    for item in result:
        source_index = item["source_index"]
        target_index = item["target_index"]
        distance_matrix[source_index][target_index] = item["distance"]
        time_matrix[source_index][target_index] = item["time"]
    
# Print matrices
print("Distance Matrix:")
print(distance_matrix)
print("\nTime Matrix:")
print(time_matrix)

