from IPython.core.display import display, HTML
import re
from utils import *
from graphics.meshmaterial import *
from simobjects.box import *
from simobjects.pointlight import *
from simobjects.pointcloud import *
import time

# ts stores the time in seconds
import httplib2
import sys


class Simulation:
    """A simulation variable."""

    _CAMERATYPE = ['perspective', 'orthographic']
    # Import the javascript code as a string

    _URL = "https://raw.githubusercontent.com/viniciusmgn/jupyterbot_vinicius/test/jupyterbot/threejs_sim.js"
    #_URL = "D:\\PycharmProjects\\pyProj\\jupyterbot\\threejs_sim.js"

    _STRJAVASCRIPT = "<html>\n"
    _STRJAVASCRIPT += "<body>\n"

    _STRJAVASCRIPT += "<div id='canvas_container_##SIMID##' style='width:##WIDTH##px;height:##HEIGHT##px;position:relative'>\n"
    _STRJAVASCRIPT += "<div id='loading_screen_##SIMID##' style='width:##WIDTH##px;height:##HEIGHT##px;position:relative; " \
                      "background-color: #19bd39;text-align:center;align-items:center;display:flex;justify-content:center'> \n "
    _STRJAVASCRIPT += "<img src='https://raw.githubusercontent.com/viniciusmgn/jupyterbot_vinicius/test/contents/SVG" \
                      "/logo_uai_bot.svg' style='width:200px;height:114px'/>\n "
    _STRJAVASCRIPT += "</div>\n"
    _STRJAVASCRIPT += "<canvas id='scene_##SIMID##' width='##WIDTH##px' height='##HEIGHT##px'></canvas>\n"
    _STRJAVASCRIPT += "<!-- USER DIVS GO HERE -->"
    _STRJAVASCRIPT += "</div>\n"
    _STRJAVASCRIPT += "\n <script type=\"module\">\n"

    _STRJAVASCRIPT += httplib2.Http().request(_URL)[1].decode()

    #for line in open("D:\\PycharmProjects\\pyProj\\jupyterbot\\threejs_sim.js").readlines():
    #    _STRJAVASCRIPT += line

    _STRJAVASCRIPT += "\n </script>"
    _STRJAVASCRIPT += "\n </body>"
    _STRJAVASCRIPT += "\n </html>"

    #######################################
    # Attributes
    #######################################

    @property
    def list_of_objects(self):
        """A list of all objects."""
        return self._list_of_objects

    @property
    def list_of_names(self):
        """A list of all object names."""
        return self._list_of_names

    @property
    def ambient_light_intensity(self):
        """A list of all object names."""
        return self._ambient_light_intensity

    @property
    def ldr_urls(self):
        """A list of the LDR light urls."""
        return self._ldr_urls

    @property
    def camera_type(self):
        """Type of the camera."""
        return self._camera_type

    @property
    def width(self):
        """Width, in pixels, of the canvas"""
        return self._width

    @property
    def height(self):
        """Height, in pixels, of the canvas"""
        return self._height

    #######################################
    # Constructor
    #######################################

    def __init__(self, obj_list=[], ambient_light_intensity=12, ldr_urls=None, camera_type="perspective", width=800,
                 height=600):

        if not Utils.is_a_number(ambient_light_intensity) or ambient_light_intensity < 0:
            raise Exception("The parameter 'ambient_light_intensity' should be a nonnegative float.")

        if not (camera_type in Simulation._CAMERATYPE):
            raise Exception("The parameter 'camera_type' must be one of the following strings: " + str(
                Simulation._CAMERATYPE) + ".")

        if not Utils.is_a_number(width) or width <= 0:
            raise Exception("The parameter 'width' must be a positive float.")

        if not Utils.is_a_number(height) or height <= 0:
            raise Exception("The parameter 'height' must be a positive float.")

        if not (ldr_urls is None):
            if not (str(type(ldr_urls)) == "<class 'list'>") or not (len(ldr_urls) == 6):
                raise Exception("The parameter 'ldr_urls' should be a list of six urls or 'None'.")
            else:
                for url in ldr_urls:
                    error = Utils.is_url_available(url, ['png', 'bmp', 'jpg', 'jpeg'])
                    if not (error == "ok!"):
                        raise Exception("The parameter 'url' " + error)

        self._list_of_objects = []
        self._list_of_names = []
        self._ambient_light_intensity = ambient_light_intensity
        self._camera_type = camera_type
        self._ldr_urls = ldr_urls
        self._width = width
        self._height = height

        if str(type(obj_list)) == "<class 'list'>":
            for obj in obj_list:
                self.add(obj)
        else:
            raise Exception("The parameter 'obj_list' should be a list of objects.")

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "Simulation: \n\n"
        string += " Variables: \n"
        string += str(self.list_of_names)
        string += " Width: "+str(self.width)+"px, Height: "+str(self.height)+"px\n"
        string += " Camera type: "+str(self.camera_type)+"\n"
        string += " Ambient light intensity: "+str(self.ambient_light_intensity)

        return string

    #######################################
    # Methods
    #######################################

    @staticmethod
    def create_sim_factory(objects):
        """
    Create an environment of a factory.
    Factory panorama taken from:
    'https://www.samrohn.com/360-panorama/chrysler-factory-detroit-usa-360-tour/chrysler-factory-360-panorama-tour-007/'

    Parameters
    ----------
    objects: list of objects that can be simulated (see Utils.IS_OBJ_SIM)
        The objects to be added to the scenario.

    Returns
    -------
    sim: 'Simulation' object
        Simulation object.
    """
        mesh_ground = MeshMaterial(
            texture_map='https://raw.githubusercontent.com/viniciusmgn/jupyterbot_vinicius/test/contents/Textures/factory_ground.png',
            roughness=1, metalness=1)

        ground = Box(name="ground", width=6, depth=6, height=0.01, htm=Utils.trn([0, 0, 0.005]),
                     mesh_material=mesh_ground)

        light1 = PointLight(name="light1", color="white", intensity=2.5, htm=Utils.trn([-1,-1, 1.5]))
        light2 = PointLight(name="light2", color="white", intensity=2.5, htm=Utils.trn([-1, 1, 1.5]))
        light3 = PointLight(name="light3", color="white", intensity=2.5, htm=Utils.trn([ 1,-1, 1.5]))
        light4 = PointLight(name="light4", color="white", intensity=2.5, htm=Utils.trn([ 1, 1, 1.5]))

        ldr_url = "https://raw.githubusercontent.com/viniciusmgn/jupyterbot_vinicius/test/contents/LDR/factory_"
        ldr_list = [ldr_url + "px.png", ldr_url + "nx.png", ldr_url + "py.png", ldr_url + "ny.png", ldr_url + "nz.png",
                    ldr_url + "nz.png"]

        sim = Simulation(objects, ambient_light_intensity=5, ldr_urls=ldr_list)
        sim.add(ground)
        sim.add(light1)
        sim.add(light2)
        sim.add(light3)
        sim.add(light4)

        return sim

    @staticmethod
    def create_sim_grid(objects):

        def points_line(limx, limy, limz, step):
            points = np.zeros((3, 0))
            for i in range(round((limx[1] - limx[0]) / step) + 1):
                x = limx[0] + step * i
                for j in range(round((limy[1] - limy[0]) / step) + 1):
                    y = limy[0] + step * j
                    for k in range(round((limz[1] - limz[0]) / step) + 1):
                        z = limz[0] + step * k
                        points = np.block([points, np.array([[x], [y], [z]])])
            return points

        def create_grid(name, limx, limy, limz, spacing, step, color, size):
            points = np.zeros((3, 0))

            for i in range(round((limx[1] - limx[0]) / spacing) + 1):
                x = limx[0] + spacing * i
                for j in range(round((limy[1] - limy[0]) / spacing) + 1):
                    y = limy[0] + spacing * j
                    points = np.block([points, points_line([x, x], [y, y], limz, step)])

            for i in range(round((limx[1] - limx[0]) / spacing) + 1):
                x = limx[0] + spacing * i
                for j in range(round((limz[1] - limz[0]) / spacing) + 1):
                    z = limz[0] + spacing * j
                    points = np.block([points, points_line([x, x], limy, [z, z], step)])

            for i in range(round((limy[1] - limy[0]) / spacing) + 1):
                y = limy[0] + spacing * i
                for j in range(round((limz[1] - limz[0]) / spacing) + 1):
                    z = limz[0] + spacing * j
                    points = np.block([points, points_line(limx, [y, y], [z, z], step)])

            return PointCloud(name, points, size, color)

        sim = Simulation(objects, camera_type="orthographic")
        sim.add(create_grid("majorXYGrid", [-2, 2], [-2, 2], [0, 0], 1, 0.01, "black", 0.2))
        sim.add(create_grid("majorXZGrid", [-2, 2], [-2, -2], [0, 4], 1, 0.01, "black", 0.2))
        sim.add(create_grid("majorYZGrid", [-2, -2], [-2, 2], [0, 4], 1, 0.01, "black", 0.2))
        sim.add(create_grid("minorXYGrid", [-2, 2], [-2, 2], [0, 0], 0.5, 0.02, "#898989", 0.03))
        sim.add(create_grid("minorXZGrid", [-2, 2], [-2, -2], [0, 4], 0.5, 0.02, "#898989", 0.03))
        sim.add(create_grid("minorYZGrid", [-2, -2], [-2, 2], [0, 4], 0.5, 0.02, "#898989", 0.03))

        return sim

    def set_size(self, width, height):
        """
    Change the size of the simulator canvas.

    Parameters
    ----------
    width : positive float
        The canvas width in pixels.
    height : positive float
        The canvas height in pixels.
    """

        if not Utils.is_a_number(width) or width <= 0:
            raise Exception("The parameter 'width' must be a positive float.")

        if not Utils.is_a_number(height) or height <= 0:
            raise Exception("The parameter 'height' must be a positive float.")

        self._width = width
        self._height = height

    def gen_code(self):
        """Generate code for injection."""

        string = Simulation._STRJAVASCRIPT

        for obj in self.list_of_objects:
            if Utils.get_jupyterbot_type(obj) == "jupyterbot.HTMLDiv":
                string = re.sub("<!-- USER DIVS GO HERE -->",
                                "<div id='" + obj.name + "'>" + obj.name + "</div>\n <!-- USER DIVS GO HERE -->",
                                string)

        string = re.sub("//USER INPUT GOES HERE",
                        "ambientLight.intensity = " + str(self.ambient_light_intensity) + ";\n //USER INPUT GOES HERE",
                        string)

        string = re.sub("//USER INPUT GOES HERE",
                        "const ldrUrls = " + (
                            str(self.ldr_urls) if not (self.ldr_urls is None) else "[]") + ";\n //USER INPUT GOES HERE",
                        string)

        string = re.sub("//SIMULATION PARAMETERS GO HERE", "const delay = 500; \n //SIMULATION PARAMETERS GO HERE",
                        string)

        max_time = 0
        for obj in self.list_of_objects:
            string = re.sub("//USER INPUT GOES HERE", obj.gen_code(), string)
            max_time = max(max_time, obj._max_time)

        sim_id = str(time.time()).replace(".","")

        string = re.sub("//SIMULATION PARAMETERS GO HERE",
                        "const maxTime = " + str(max_time) + "; \n //SIMULATION PARAMETERS GO HERE",
                        string)
        string = re.sub("//SIMULATION PARAMETERS GO HERE",
                        "const cameraType= '" + self.camera_type + "'; \n //SIMULATION PARAMETERS GO HERE",
                        string)
        string = re.sub("//SIMULATION PARAMETERS GO HERE",
                        "const sceneID= '" + sim_id + "'; \n //SIMULATION PARAMETERS GO HERE",
                        string)

        string = re.sub("##WIDTH##", str(self.width), string)
        string = re.sub("##HEIGHT##", str(self.height), string)
        string = re.sub("##HEIGHTLOGO##", str(round(0.57*self.width)), string)
        string = re.sub("##SIMID##", sim_id, string)

        return string

    def run(self):
        """Run simulation."""

        display(HTML(self.gen_code()))

    def save(self, address, file_name):

        file = open(address + "/" + file_name + ".html", "w+")
        file.write(self.gen_code())
        file.close()

    def add(self, obj_sim):
        """
    Add an object to the simulation. It should be an object that
    can be simulated (Utils.is_a_obj_sim(obj) is true).

    Parameters
    ----------
    obj_sim : object or list of object
        The object(s) to be added to simulation.
    """

        if str(type(obj_sim)) == "<class 'list'>":
            for obj in obj_sim:
                self.add(obj)
        else:
            # Error handling
            if not Utils.is_a_obj_sim(obj_sim):
                raise Exception("The parameter 'obj' should be one of the following: " + str(Utils.IS_OBJ_SIM) + ".")

            if obj_sim.name in self.list_of_names:
                raise Exception("The name '" + obj_sim.name + "' is already in the list of symbols.")

            # end error handling

            self._list_of_names.append(obj_sim.name)
            self._list_of_objects.append(obj_sim)
