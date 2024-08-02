from filter import *

prefix = os.getcwd() + "/lidar_data/"
smap_cloud_path = prefix + '/S/2024-03-06_cloud.txt'

smapCloud = o3d.io.read_point_cloud(smap_cloud_path, format='xyz')
smapCloud = remove_mapPoint(smapCloud)

o3d.visualization.draw_geometries_with_editing([smapCloud])

