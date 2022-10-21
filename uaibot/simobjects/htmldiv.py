from utils import *


class HTMLDiv:
    """
  A HTML div that is used to display a text into the scenario canvas.

  Parameters
  ----------
  name : string
      The object's name.
      (default: 'genHTMLDiv').

  html_text : string
      The html string.
      (default: 'Text').

  style : string
      CSS style string  (example: 'position:absolute;top:200px,right:100px').
      (default: 'Text').
  """

    #######################################
    # Attributes
    #######################################

    @property
    def name(self):
        """The name of the object."""
        return self._name

    @property
    def html_text(self):
        """The HTML string representing the text. HTML markup is allowed here."""
        return self._html_text

    @property
    def style(self):
        """The CSS style string."""
        return self._style

    #######################################
    # Constructor
    #######################################

    def __init__(self, name="", html_text="Text", style=""):

        # Error handling

        if name=="":
            name="var_htmldiv_id_"+str(id(self))

        if not (Utils.is_a_name(name)):
            raise Exception(
                "The parameter 'name' should be a string. Only characters 'a-z', 'A-Z', '0-9' and '_' are allowed. "
                "It should not begin with a number.")

        if not str(type(html_text)) == "<class 'str'>":
            raise Exception("The parameter 'html_text' should be a string.")

        if not str(type(style)) == "<class 'str'>":
            raise Exception("The parameter 'style' should be a string.")

        # end error handling

        self._name = name
        self._html_text = html_text
        self._style = style
        self._frames = []
        self._max_time = 0

        self.set_ani_frame(self.html_text)

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "Text with name '" + self.name + "': \n\n"
        string += " HTML string: " + str(self.html_text) + "\n"
        string += " Style: " + str(self.style) + "\n"

        return string

    #######################################
    # Methods
    #######################################

    def add_ani_frame(self, time, html_text=None, style=None):
        """

    """
        if html_text is None:
            html_text = self.html_text
        if style is None:
            style = self.style

        # Error handling
        if not str(type(html_text)) == "<class 'str'>":
            raise Exception("The parameter 'html_text' should be either a string or 'None'.")

        if not str(type(style)) == "<class 'str'>":
            raise Exception("The parameter 'style' should be a string.")
        # end error handling

        self._html_text = html_text
        self._style = style
        self._max_time = max(self._max_time, time)

        self._frames.append([time, html_text, style])

    # Set config. Restart animation queue
    def set_ani_frame(self, html_text=None, style=None):
        """

    """
        if html_text is None:
            html_text = self.html_text
        if style is None:
            style = self.style

        # Error handling
        if not str(type(html_text)) == "<class 'str'>":
            raise Exception("The parameter 'html_text' should be either a string or 'None'.")

        if not str(type(style)) == "<class 'str'>":
            raise Exception("The parameter 'style' should be a string.")

        # end error handling

        self._frames = []
        self.add_ani_frame(0, html_text, style)
        self._max_time = 0

    def gen_code(self):
        """Generate code for injection."""

        string = "\n"
        string += "//BEGIN DECLARATION OF THE HTML DIV '" + self.name + "'\n\n"
        string += "const var_" + self.name + " = new HTMLDiv('" + self.name + "', " + str(self._frames) + ");\n"
        string += "sceneElements.push(var_" + self.name + ");\n"
        string += "//USER INPUT GOES HERE"

        return string
