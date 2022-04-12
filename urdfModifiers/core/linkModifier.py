from dataclasses import dataclass
from operator import length_hint
from urdfpy import xyz_rpy_to_matrix, matrix_to_xyz_rpy
from urdfModifiers.core import modifier
import math
import numpy as np
from urdfModifiers.geometry import * 
from urdfpy import Cylinder as cylinderUrdfPy
from urdfpy import Box as boxUrdfPy 
from urdfpy import Sphere as sphereUrdfPy 

@dataclass
class LinkModifier(modifier.Modifier):
    """Class to modify links in a URDF"""
    def __init__(self, link, axis = None):
        super().__init__(link, geometry.RobotElement.LINK)
        geometry_type, _ = self.get_geometry(self.get_visual())
        self.axis = axis

    @classmethod
    def from_name(cls, link_name, robot, axis = None):
        """Creates an instance of LinkModifier by passing the robot object and link name"""
        return cls(LinkModifier.get_element_by_name(link_name, robot), axis)

    @staticmethod
    def get_element_by_name(link_name, robot):
        """Explores the robot looking for the link whose name matches the first argument"""
        link_list = [corresponding_link for corresponding_link in robot.links if corresponding_link.name == link_name]
        if len(link_list) != 0:
            return link_list[0]
        else:
            return None

    def modify(self, modifications):
        """Performs the dimension and density modifications to the current link"""
        original_density = self.calculate_density()
        original_radius = self.get_radius()
        original_length = self.get_significant_length()
        original_mass = self.get_mass()
        if modifications.radius:
            geometry_type, _ = self.get_geometry(self.get_visual())
            if geometry_type == geometry.Geometry.BOX:
                raise Exception('Cannot modify radius of box geometry')
            if modifications.radius.absolute:
                self.set_radius(modifications.radius.value)
            else:
                if original_radius is not None:
                    self.set_radius(original_radius * modifications.radius.value)
        if modifications.dimension:
            geometry_type, _ = self.get_geometry(self.get_visual())
            if geometry_type == geometry.Geometry.SPHERE:
                raise Exception('Cannot modify length of sphere geometry')
            if modifications.dimension.absolute:
                self.set_length(modifications.dimension.value)
            else:
                if original_length is not None:
                    self.set_length(original_length * modifications.dimension.value)
        if modifications.density:
            if modifications.density.absolute:
                self.set_density(modifications.density.value)
            else:
                self.set_density(original_density * modifications.density.value)
        if modifications.mass:
            if modifications.mass.absolute:
                self.set_mass(modifications.mass.value)
            else:
                self.set_mass(original_mass * modifications.mass.value)
        if modifications.position:
            original_position = self.get_origin_position()
            if modifications.position.absolute:
                self.set_origin_position(modifications.position.value)
            else:
                self.set_origin_position(original_position * modifications.position.value)
        if modifications.geometry_type: 
            self.set_geometry_type(modifications.geometry_type, modifications.geometry_type_dimension)
            self.set_density(original_density)
        self.update_inertia()


    def get_visual(self):
        """Returns the visual object of a link"""
        return self.element.visuals[0]

    def get_collision(self):
        """Returns the collision object of a link"""
        return (self.element.collisions[0] if self.element.collisions else None)

    def get_significant_length(self):
        """Gets the significant length for a cylinder or box geometry"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.BOX):
            if (self.axis is not None):
                if (self.axis == geometry.Side.X):
                    return visual_data.size[0]
                elif (self.axis == geometry.Side.Y):
                    return visual_data.size[1]
                elif (self.axis == geometry.Side.Z):
                    return visual_data.size[2]
            else:
                raise Exception(f"Error getting length for link {self.element.name}'s volume: Box geometry with no axis")
        elif (geometry_type == geometry.Geometry.CYLINDER):
            return visual_data.length
        elif (geometry_type == geometry.Geometry.SPHERE):
            return visual_data.radius
        else:
            return None

    def get_radius(self):
        """Returns the radius if the link geometry is cylinder or sphere and None otherwise"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        return visual_data.radius if geometry_type == geometry.Geometry.CYLINDER or geometry_type == geometry.Geometry.SPHERE else None

    def set_radius(self, new_radius):
        """Sets the radius of a link if its geometry is cylider or sphere"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.CYLINDER or geometry_type == geometry.Geometry.SPHERE):
            visual_data.radius = new_radius
        geometry_type_collision, visual_data_collision = self.get_geometry(self.get_collision())
        if geometry_type_collision and (geometry_type_collision == geometry.Geometry.CYLINDER or geometry_type_collision == geometry.Geometry.SPHERE):
            visual_data_collision.radius = new_radius

    def set_length(self, length):
        """Modifies a link's length, in a manner that is logical with its geometry"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        if (geometry_type == geometry.Geometry.BOX):
            if (self.axis is not None):
                if (self.axis == geometry.Side.X):
                    visual_data.size[0] = length
                elif (self.axis == geometry.Side.Y):
                    visual_data.size[1] = length
                elif (self.axis == geometry.Side.Z):
                    visual_data.size[2] = length
            else:
                raise Exception(f"Error modifying link {self.element.name}'s volume: Box geometry with no axis")
        elif (geometry_type == geometry.Geometry.CYLINDER):
            visual_data.length = length
        geometry_type_collision, visual_data_collision = self.get_geometry(self.get_collision())
        if (geometry_type_collision == geometry.Geometry.BOX):
            if (self.axis is not None):
                if (self.axis == geometry.Side.X):
                    visual_data_collision.size[0] = length
                elif (self.axis == geometry.Side.Y):
                    visual_data_collision.size[1] = length
                elif (self.axis == geometry.Side.Z):
                    visual_data_collision.size[2] = length
            else:
                raise Exception(f"Error modifying link {self.element.name}'s volume: Box geometry with no axis")
        elif (geometry_type_collision == geometry.Geometry.CYLINDER):
            visual_data_collision.length = length

    def get_origin_position(self):
        visual_object = self.get_visual()
        if (self.axis is not None):
            if (self.axis == geometry.Side.X):
                return matrix_to_xyz_rpy(visual_object.origin)[0]
            elif (self.axis == geometry.Side.Y):
                return matrix_to_xyz_rpy(visual_object.origin)[1]
            elif (self.axis == geometry.Side.Z):
                return matrix_to_xyz_rpy(visual_object.origin)[2]
        else:
            raise Exception(f"Error modifying link {self.element.name}'s position: no axis")

    def set_origin_position(self, value):
        visual_object = self.get_visual()
        collision_object = self.get_collision()
        inertia = self.element.inertial

        origin = matrix_to_xyz_rpy(visual_object.origin)
        # origin[3:] = [0.0,1.6,0.0]
        # origin[3:] = [1.6, 0.0, 0.0]
        if (self.axis is not None):
            if (self.axis == geometry.Side.X):
                origin[0] = value
            elif (self.axis == geometry.Side.Y):
                origin[1] = value
            elif (self.axis == geometry.Side.Z):
                origin[2] = value
            visual_object.origin = xyz_rpy_to_matrix(origin)
            if (collision_object is not None):
                collision_object.origin = xyz_rpy_to_matrix(origin)
            if (inertia is not None):
                inertia.origin = xyz_rpy_to_matrix(origin)
        else:
            raise Exception(f"Error modifying link {self.element.name}'s position: no axis")

    def modify_origin_rpy(self,rpy): 
        visual_object = self.get_visual()
        origin = matrix_to_xyz_rpy(visual_object.origin)
        # origin[3:] = [0.0,1.6,0.0]
        # origin[3:] = [1.6, 0.0, 0.0]
        origin[3:] = rpy
        visual_object.origin = xyz_rpy_to_matrix(origin)
    
    def set_geometry_type(self, geometry_type, dimension): 
        new_lenght = 0.0
        new_width = 0.0
        old_type, old_dimension  = self.get_geometry(self.get_visual())
        if(old_type == geometry.Geometry.SPHERE): 
            new_lenght = 2*old_dimension.radius
            new_width = 2*old_dimension.radius
        elif(old_type == geometry.Geometry.CYLINDER): 
            new_lenght = 2*old_dimension.radius-0.01
            new_width = 2*old_dimension.radius-0.01
        elif(old_type == geometry.Geometry.BOX): 
            new_lenght = old_dimension.size[0]
            new_width = old_dimension.size[1]
        if(geometry_type == geometry.Geometry.BOX): 
            print(self.get_visual())
            dimension_new = [new_lenght, new_width, dimension]
            print(dimension_new)
            visuals_new = boxUrdfPy(dimension_new)
            # visuals_new.size = dimension
            self.element.visuals[0].geometry.box = visuals_new
            self.element.visuals[0].geometry.cylinder = None
            self.element.visuals[0].geometry.sphere = None
            old_origin = self.get_origin_position()
            self.set_origin_position(dimension/2*math.copysign(1,old_origin))
            print(self.get_visual())
        elif(geometry_type == geometry.Geometry.CYLINDER): 
            
            length = dimension
            radius = 0.075/2
            
            visuals_new = cylinderUrdfPy(radius=radius, length=length)
            print("radius", radius, "length", length)
            print("radius", visuals_new.radius, 'length', visuals_new.length)
            self.element.visuals[0].geometry.cylinder = visuals_new
            print('radius', self.element.visuals[0].geometry.cylinder.radius)
            print("lenght", self.element.visuals[0].geometry.cylinder.length)
            self.element.visuals[0].geometry.sphere = None
            self.element.visuals[0].geometry.box = None
            old_origin = self.get_origin_position()
            self.set_origin_position(0.0)
        elif(geometry_type == geometry.Geometry.SPHERE): 
            visuals_new = sphereUrdfPy
            visuals_new.radius = dimension[0]
            self.element.visuals[0].geometry.sphere = visuals_new
            self.element.visuals[0].geometry.box = None
            self.element.visuals[0].geometry.cylinder = None 

        return   


    @staticmethod
    def get_visual_static(link):
        """Static method that returns the visual of a link"""
        return link.visuals[0]

    @staticmethod
    def get_geometry(geometry_holder):
        """Returns the geometry type and the corresponding geometry object for a given geometry holder (visual/collision)"""
        if geometry_holder is None:
            return [None, None]
        if (geometry_holder.geometry.box is not None):
            return [geometry.Geometry.BOX, geometry_holder.geometry.box]
        if (geometry_holder.geometry.cylinder is not None):
            return [geometry.Geometry.CYLINDER, geometry_holder.geometry.cylinder]
        if (geometry_holder.geometry.sphere is not None):
            return [geometry.Geometry.SPHERE, geometry_holder.geometry.sphere]
        

    def calculate_volume(self, geometry_type, visual_data):
        """Calculates volume with the formula that corresponds to the geometry"""
        if (geometry_type == geometry.Geometry.BOX):
            return visual_data.size[0] * visual_data.size[1] * visual_data.size[2]
        elif (geometry_type == geometry.Geometry.CYLINDER):
            return math.pi * visual_data.radius ** 2 * visual_data.length
        elif (geometry_type == geometry.Geometry.SPHERE):
            return 4 * math.pi * visual_data.radius ** 3 / 3

    def get_mass(self):
        """Returns the link's mass"""
        return self.element.inertial.mass

    def set_mass(self, new_mass):
        """Sets the mass value to a new value"""
        self.element.inertial.mass = new_mass

    def calculate_density(self):
        """Calculates density from mass and volume"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        return self.get_mass() / self.calculate_volume(geometry_type, visual_data)

    def set_density(self, density):
        """Changes the mass of a link by preserving a given density."""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        volume = self.calculate_volume(geometry_type, visual_data)
        self.element.inertial.mass = volume * density

    def calculate_inertia(self):
        """Calculates inertia (ixx, iyy and izz) with the formula that corresponds to the geometry
        Formulas retrieved from https://en.wikipedia.org/wiki/List_of_moments_of_inertia"""
        geometry_type, visual_data = self.get_geometry(self.get_visual())
        mass = self.get_mass()
        if (geometry_type == geometry.Geometry.BOX):
            return mass / 12 * np.array([visual_data.size[1] ** 2 + visual_data.size[2] ** 2, 
                                visual_data.size[0] ** 2 + visual_data.size[2] ** 2,
                                visual_data.size[0] ** 2 + visual_data.size[1] ** 2])
        elif (geometry_type == geometry.Geometry.CYLINDER):
            i_xy_incomplete = (3 * visual_data.radius ** 2 + visual_data.length ** 2) / 12
            return mass * np.array([i_xy_incomplete, i_xy_incomplete, visual_data.radius ** 2 / 2])
        elif (geometry_type == geometry.Geometry.SPHERE):
            inertia = 2 * mass * visual_data.radius ** 2 / 5
            return np.array([inertia, inertia, inertia])

    def update_inertia(self):
        """Updates the inertia of a link to match its volume and mass."""
        if (self.element.inertial is not None):
            inertia = self.element.inertial.inertia
            new_inertia = self.calculate_inertia()
            new_inertia[new_inertia < 0.01] = 0.01
            for i in range(3):
                for j in range(3):
                    if (i == j):
                        inertia[i,j] = new_inertia[i]
                    else:
                        inertia[i,j] = 0

    def __str__(self):
        return f"Link modifier with name {self.element.name}, origin modifier {self.origin_modifier}, axis {self.axis}"
