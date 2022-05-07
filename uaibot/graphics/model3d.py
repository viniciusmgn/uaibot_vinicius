from utils import *
from graphics.meshmaterial import *
import numpy as np




class Model3D:
    """
   A class that represents a 3d model of an object.

   Parameters
   ----------
   url : string
      The url that contains the 3d object.
      It must have one of the following formats: 'obj'.

   scale : positive float
       The scaling parameter of the object.
       (default: 1).

   htm : 4x4 numpy array or 4x4 nested list
       The htm of the 3d models. This is used to tune the 'default' htm for the object in the uaibot simulator.
       This is necessary because the 3d model can have a different 'default' pose than the desired one.
       (default: np.identity(4)).

   mesh_material : 'uaibot.MeshMaterial' object
       The mesh_material to be applied into the 3d model.
       (default: None).
   """

    #######################################
    # Attributes
    #######################################

    @property
    def url(self):
        """The 3d model url."""
        return self._url

    @property
    def scale(self):
        """The object scale."""
        return self._scale

    @property
    def htm(self):
        """Object pose. A 4x4 homogeneous transformation matrix written is scenario coordinates."""
        return self._htm

    @property
    def mesh_material(self):
        """The model mesh material."""
        return self._mesh_material

    #######################################
    # Constructor
    #######################################

    def __init__(self, url="", scale=1, htm=np.identity(4), mesh_material=None):

        # Error handling

        error = Utils.is_url_available(url, ['obj','stl','dae'])
        if not (error == "ok!"):
            raise Exception("The parameter 'url' " + error)

        if not Utils.is_a_matrix(htm, 4, 4):
            raise Exception("The parameter 'htm' should be a 4x4 homogeneous transformation matrix.")

        if not Utils.is_a_number(scale) or scale < 0:
            raise Exception("The parameter 'scale' should be a float.")

        if not (Utils.get_uaibot_type(mesh_material) == "uaibot.MeshMaterial" or (mesh_material is None)):
            raise Exception("The parameter 'mesh_material' should be a 'uaibot.MeshMaterial' object or 'None'.")

        # end error handling

        self._url = url
        self._scale = scale
        self._htm = htm
        self._type = url[url.rfind(".")+1:len(url)+1]


        if mesh_material is None:
            self._mesh_material = MeshMaterial()
        else:
            self._mesh_material = mesh_material

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "3D object model with url: '" + self.url + "': \n\n"

        return string

    #######################################
    # Methods
    #######################################

    def gen_code(self, name):
        """Generate code for injection."""

        string = self.mesh_material.gen_code(name)
        string += "const htm_" + name + " = new Matrix4(); \n"
        string += "htm_" + name + ".set(" \
                  + str(self.htm[0,0]) + "," + str(self.htm[0,1]) + "," + str(self.htm[0,2]) + "," + str(
            self.htm[0,3]) + "," \
                  + str(self.htm[1,0]) + "," + str(self.htm[1,1]) + "," + str(self.htm[1,2]) + "," + str(
            self.htm[1,3]) + "," \
                  + str(self.htm[2,0]) + "," + str(self.htm[2,1]) + "," + str(self.htm[2,2]) + "," + str(
            self.htm[2,3]) + "," \
                  + str(self.htm[3,0]) + "," + str(self.htm[3,1]) + "," + str(self.htm[3,2]) + "," + str(
            self.htm[3,3]) + ");\n"

        string += "const object3d_" + name + "={\n"
        string += "url: '" + self.url + "',\n"
        string += "scale: " + str(self.scale) + ",\n"
        string += "matrix: htm_" + name + ",\n"
        string += "type: '" + self._type + "',\n"
        string += "mesh_material: material_" + name + "};\n\n"

        return string
