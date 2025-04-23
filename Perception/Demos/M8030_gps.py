#!interpreter
# -*- coding: utf-8 -*-

"""
Demo of GPS module data and visualization on map.
"""

# ######### Built-in/Generic imports ##########
from typing import Tuple, Type, Dict
from asammdf import MDF
import folium

__author__ = 'Nico Hessenthaler'
__copyright__ = 'Copyright 2023, HHN - Autonomous Systems: Perception and Situation Understanding'
__credits__ = ['-']
__license__ = '-'
__version__ = '1.0.0'
__maintainer__ = 'Nico Hessenthaler'
__email__ = 'nico.hessenthaler@gmail.com'
__status__ = 'Finished'

MF4_FILE_PATH = "C:\\Users\\nicoh\\Documents\\Daten\\02_Nico\\06_Lehrauftrag\\00_HHN\\00_Situation_and_Perception_Analysis\\Vorlesung\\Vorlesung\\Perc02\\Messungen\\PI_FAS_23_10_04_2023_13_47_29.mf4"


# ######### Source code ##########

def load_mf4_file(file_path: str) -> Type[MDF.iter_channels]:
    """
    Method that loads a selected MF4 file from a file path.

    Args:
        file_path (str): Path to the measurement file to open containing the GPS coordinates.

    Returns:
        mf4_channels (MDF.iter_channels): Channels of the mf4 iterator.
    """

    # Open file and read channels
    mf4_handle = MDF(file_path)
    mf4_channels = mf4_handle.iter_channels()
    
    return mf4_channels


def channels_to_dict(mf4_channels: Type[MDF.iter_channels]) -> Dict:
    """
    Method that re-arranges all channels from an iterator to a dict of timestamps and values.

    Args:
        mf4_channels (MDF.iter_channels): Channels of the mf4 iterator.

    Returns:
        measurement_data_dict (dict): Dictionary of timestamps and samples for all channels.
    """

    measurement_data_dict = {}

    for channel in mf4_channels:
        measurement_data_dict[channel.name] = \
            {"samples": list(channel.samples), "timestamps": list(channel.timestamps)}
    
    return measurement_data_dict


def add_map_marker(gps_coordinates: Tuple[float, float]) -> Type[folium.Map]:
    """
    Method that adds the captured GPS marker to a map and displays it.

    Args:
        gps_coordinates (Tuple): Tuple of GPS coordinates in longitude, latitude.

    Returns:
        map (folium.Map): Annotated folium map containing markers.
    """

    # Define start location of the map and zoom level
    map = folium.Map((gps_coordinates[0][0], gps_coordinates[0][1]), zoom_start=15)

    # Plot the markers on the map
    # Latitude, longitude
    for pt in gps_coordinates:
        marker = folium.Marker([pt[0], pt[1]]) 
        map.add_child(marker)

    return map


def add_polyline(gps_coordinates: Tuple[float, float],\
                 map: Type[folium.Map]) -> Type[folium.Map]:
    """
    Method that adds the captured GPS marker to a map and displays it.

    Args:
        gps_coordinates (Tuple): Tuple of GPS coordinates in longitude, latitude.
        map (folium.Map): Annotated folium map containing markers.

    Returns:
        map (folium.Map): Annotated folium map containing markers and polylines.
    """

    folium.PolyLine(gps_coordinates, tooltip="Driven path").add_to(map) 

    return map


def show_map(map: Type[folium.Map]) -> None:
    """
    Function that shows the generated map with markers and polyline in the default web browser.

    Args:
        map (folium.Map): Annotated folium map containing markers and polylines.

    Returns:
        ():
    """

    # Show the map in the default web browser
    map.show_in_browser()

    return


# Main function
if __name__=="__main__": 

    # Read the MF4 file containing the GPS coordinates
    mf4_channels = load_mf4_file(MF4_FILE_PATH)

    # Convert the channels to a dict
    measurement_data_dict = channels_to_dict(mf4_channels)

    # Generate pairs of longitude / latitude values for plotting
    gps_coordinates = list(zip(measurement_data_dict["GPS_latitude"]["samples"], \
                               measurement_data_dict["GPS_longitude"]["samples"]))
    
    # Add single markers on map
    map = add_map_marker(gps_coordinates)

    # Add polyline of the path on the map
    map = add_polyline(gps_coordinates, map)

    # Display the map in the browser
    show_map(map)
    
#EOF
