#!/usr/bin/env python3
import os
import sys
import yaml
import xacro
from ament_index_python import get_package_share_directory
import subprocess
from lxml import etree

def remove_comments_from_xml(xml_string):
    parser =  etree.XMLParser(remove_comments=True)
    tree= etree.fromstring(xml_string, parser = parser)
    return etree.tostring(tree, encoding='utf8', method='xml').decode()

def spawn_shapes(world_name):

    with open(os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'config', 'shapes_spawner.yaml'), "r") as stream:
        try:
            shapes_spawner_config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    
    request = 'data: ['
    flag = False
    for shape_type in ['boxes', 'spheres', 'cylinders']:
        if shapes_spawner_config[shape_type] is None:
            continue
        for shape in shapes_spawner_config[shape_type]:
            
            if(shape_type == 'boxes'):
                xacro_file = os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'models', 'templates', 'simple_box.xacro')  
                assert os.path.exists(xacro_file), "The simple_box.xacro doesnt exist in "+str(xacro_file) 
                shape_description_config = xacro.process_file(xacro_file, mappings={'wx' : str(shape[0]),
                    'wy' : str(shape[1]),
                    'wz' : str(shape[2]),
                    'link_name': shape[7],
                    'color': shape[8]
                    })
            elif(shape_type == 'spheres'):
                xacro_file = os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'models', 'templates', 'simple_sphere.xacro')  
                assert os.path.exists(xacro_file), "The simple_sphere.xacro doesnt exist in "+str(xacro_file) 
                shape_description_config = xacro.process_file(xacro_file, mappings={'radius' : str(shape[0]),
                    'link_name': shape[5],
                    'color': shape[6]
                    })
            elif(shape_type == 'cylinders'):
                xacro_file = os.path.join(get_package_share_directory('sandwich_bot_2_0_ign_simulation'), 'models', 'templates', 'simple_cylinder.xacro')  
                assert os.path.exists(xacro_file), "The simple_cylinder.xacro doesnt exist in "+str(xacro_file) 
                shape_description_config = xacro.process_file(xacro_file, mappings={'radius' : str(shape[0]),
                    'height' : str(shape[1]),
                    'link_name': shape[6],
                    'color': shape[7]
                    })

            shape_desc = '<?xml version=\'1.0\'?>' + remove_comments_from_xml(shape_description_config.toxml())
            shape_desc = shape_desc.replace('"','\'')
            if(shape_type == 'boxes'):
                [x_index, y_index, z_index, model_name_index ] =  [3,4,5,6]
            elif(shape_type == 'spheres'):
                [x_index, y_index, z_index, model_name_index ] =  [1,2,3,4]
            elif(shape_type == 'cylinders'):
                [x_index, y_index, z_index, model_name_index ] =  [2,3,4,5]

            temp_str = f'{{sdf: "{shape_desc}" pose: {{position: {{x: {shape[x_index]},y: {shape[y_index]},z: {shape[z_index]}}}}} name: "{shape[model_name_index]}" allow_renaming: true}}'
            if(flag == False):
                request = request + temp_str
                flag = True
            else:
                request = request+ ',' + temp_str
    request = request + ']'

    subprocess.run(["ign", 
            "service", 
            "-s", 
            "/world/"+world_name+"/create_multiple", 
            "--reqtype", 
            "ignition.msgs.EntityFactory_V", 
            "--reptype", 
            "ignition.msgs.Boolean", 
            "--timeout", 
            "300",
            "--req",
            request
        ])


def main():
    spawn_shapes(sys.argv[1])

if __name__== "__main__":
    main()